#include <dynamic_introspection/dynamic_introspection.h>
#include <dynamic_introspection/BoolParameter.h>
#include <dynamic_introspection/DoubleParameter.h>
#include <dynamic_introspection/IntParameter.h>
#include <dynamic_introspection/MarkerParameter.h>
#include <exception>

template <typename T>
typename std::vector<T>::iterator const_iterator_cast(std::vector<T> &v,
                                                      typename std::vector<T>::const_iterator iter)
{
  return v.begin() + (iter - v.cbegin());
}

const std::string getRegisteredVariables(const DynamicIntrospection *di)
{
  std::stringstream ss;

  ss << "registered ints:" << std::endl;
  for (size_t i = 0; i < di->getDataPtr()->registeredInt_.size(); ++i)
  {
    ss << "    " << std::get<0>(di->getDataPtr()->registeredInt_[i]) << std::endl;
  }
  ss << "registered double:" << std::endl;
  for (size_t i = 0; i < di->getDataPtr()->registeredDouble_.size(); ++i)
  {
    ss << "    " << std::get<0>(di->getDataPtr()->registeredDouble_[i]) << std::endl;
  }
  ss << "registered bool:" << std::endl;
  for (size_t i = 0; i < di->getDataPtr()->registeredBool_.size(); ++i)
  {
    ss << "    " << std::get<0>(di->getDataPtr()->registeredBool_[i]) << std::endl;
  }
  ss << "registered markers:" << std::endl;
  for (size_t i = 0; i < di->getDataPtr()->registeredMarkers_.size(); ++i)
  {
    ss << "    " << std::get<0>(di->getDataPtr()->registeredMarkers_[i]) << std::endl;
  }

  ROS_ERROR_STREAM(ss.str());
  return ss.str();
}

struct ExistingVariableException : public std::runtime_error
{
  ExistingVariableException(const std::string variable, const DynamicIntrospection *di)
    : std::runtime_error(""), variable_(variable), di_(di)
  {
  }

  const char *what() const throw()
  {
    std::stringstream msg;
    msg << "Registering an existing variable: " << variable_ << std::endl;
    msg << getRegisteredVariables(di_);
    ROS_ERROR_STREAM(msg.str());
    return msg.str().c_str();
  }

  const std::string variable_;
  const DynamicIntrospection *di_;
};

struct DoesNotExistingVariableException : public std::runtime_error
{
  DoesNotExistingVariableException(const std::string variable, DynamicIntrospection *di)
    : std::runtime_error(""), variable_(variable), di_(di)
  {
  }

  const char *what() const throw()
  {
    std::stringstream msg;
    msg << "Trying to delete a variable that is not registered: " << variable_ << std::endl;
    msg << getRegisteredVariables(di_);

    ROS_ERROR_STREAM(msg.str());
    return msg.str().c_str();
  }

  const std::string variable_;
  const DynamicIntrospection *di_;
};

template <typename C>
bool contains(const std::vector<std::tuple<std::string, const C *, C> > &c, const std::string &e)
{
  for (size_t i = 0; i < c.size(); ++i)
  {
    if (std::get<0>(c[i]) == e)
    {
      return true;
    }
  }
  return false;
}

template <class T>
int indexVector(std::vector<T> v, const std::string &id)
{
  for (auto it = v.begin(); it != v.end(); ++it)
  {
    if (std::get<0>(*it) == id)
    {
      return std::distance(v.begin(), it);
    }
  }

  return -1;
}

DynamicIntrospection *DynamicIntrospection::m_pInstance = NULL;

DynamicIntrospection *DynamicIntrospection::Instance()
{
  if (!m_pInstance)
  {  // Only allow one instance of class to be generated.
    m_pInstance = new DynamicIntrospection;
  }
  return m_pInstance;
}

DynamicIntrospection::DynamicIntrospection()
{
  node_handle_ = ros::NodeHandle();
  introspectionPub_ = node_handle_.advertise<dynamic_introspection::IntrospectionMsg>(
      "/introspection_data", 10);
  thread_ = boost::thread(&DynamicIntrospection::publishDataTopic, this);
}

void DynamicIntrospection::setOutputTopic(const std::string &outputTopic)
{
  introspectionPub_.shutdown();
  introspectionPub_ =
      node_handle_.advertise<dynamic_introspection::IntrospectionMsg>(outputTopic, 10);
}

/// @todo how to propperly destroy singleton patterm?
DynamicIntrospection::~DynamicIntrospection()
{
  introspectionPub_.shutdown();
  closeBag();
  delete m_pInstance;
  m_pInstance = 0;
}

void DynamicIntrospection::openBag(std::string fileName)
{
  bag_.open(fileName, rosbag::bagmode::Write);
  openedBag_ = true;
}

void DynamicIntrospection::closeBag()
{
  bag_.close();
  openedBag_ = false;
}

void DynamicIntrospection::publishDataBag()
{
  if (!openedBag_)
  {
    ROS_ERROR_STREAM("Bag is not open");
  }

  if (trylock())
  {
    registered_data_.copy();
    generateMessage();
    unlock();
    bag_.write("/introspection_data", ros::Time::now(), introspectionMessage_);
  }
}

void DynamicIntrospection::publishDataTopic()
{
  boost::unique_lock<boost::mutex> lck(updated_cond__mutex_);
  while (true)
  {
    updated_cond_.wait(lck);

    lock();
    if (introspectionPub_.getNumSubscribers() > 0)
    {
      generateMessage();
      introspectionPub_.publish(introspectionMessage_);
    }
    unlock();
  }
}

void DynamicIntrospection::publishDataTopicRT()
{
  if (trylock())
  {
    registered_data_.copy();
    unlock();
    updated_cond_.notify_one();
  }
  else
  {
    ROS_DEBUG_STREAM("Missed introspection update cycle");
  }
}


void DynamicIntrospection::generateMessage()
{
  introspectionMessage_.header.stamp = ros::Time::now();

  introspectionMessage_.ints.resize(registered_data_.registeredInt_.size());
  introspectionMessage_.doubles.resize(registered_data_.registeredDouble_.size());
  introspectionMessage_.bools.resize(registered_data_.registeredBool_.size());
  introspectionMessage_.markers.resize(registered_data_.registeredMarkers_.size());

  for (size_t i = 0; i < registered_data_.registeredInt_.size(); ++i)
  {
    dynamic_introspection::IntParameter &ip = introspectionMessage_.ints[i];
    ip.name = std::get<0>(registered_data_.registeredInt_[i]);
    ip.value = std::get<2>(registered_data_.registeredInt_[i]);
  }

  for (size_t i = 0; i < registered_data_.registeredDouble_.size(); ++i)
  {
    assert(registered_data_.registeredDouble_.size() == introspectionMessage_.doubles.size());
    dynamic_introspection::DoubleParameter &dp = introspectionMessage_.doubles[i];
    dp.name = std::get<0>(registered_data_.registeredDouble_[i]);
    dp.value = std::get<2>(registered_data_.registeredDouble_[i]);
  }

  for (size_t i = 0; i < registered_data_.registeredBool_.size(); ++i)
  {
    dynamic_introspection::BoolParameter &bp = introspectionMessage_.bools[i];
    bp.name = std::get<0>(registered_data_.registeredBool_[i]);
    bp.value = std::get<2>(registered_data_.registeredBool_[i]);
  }

  for (size_t i = 0; i < registered_data_.registeredMarkers_.size(); ++i)
  {
    dynamic_introspection::MarkerParameter &vp = introspectionMessage_.markers[i];
    vp.name = std::get<0>(registered_data_.registeredMarkers_[i]);
    vp.value = std::get<2>(registered_data_.registeredMarkers_[i]);
  }
}

void DynamicIntrospection::registerVariable(const int *variable, const std::string &id,
                                            std::vector<std::string> &registeded_ids)
{
  lock();
  if (contains(registered_data_.registeredInt_, id) ||
      contains(registered_data_.registeredDouble_, id) ||
      contains(registered_data_.registeredBool_, id) ||
      contains(registered_data_.registeredMarkers_, id))
  {
#if !defined(NDEBUG)
    throw ExistingVariableException(id, this);
#else
    ROS_ERROR_STREAM("Registering an existing variable: " << id);
#endif
  }
  else
  {
    ROS_DEBUG_STREAM("Registered int: " << id);
    std::tuple<const std::string, const int *, int> p(id, variable, 0);
    registered_data_.registeredInt_.push_back(p);
    registeded_ids.push_back(id);
  }
  unlock();
}

void DynamicIntrospection::registerVariable(const double *variable, const std::string &id,
                                            std::vector<std::string> &registeded_ids)
{
  lock();
  if (contains(registered_data_.registeredInt_, id) ||
      contains(registered_data_.registeredDouble_, id) ||
      contains(registered_data_.registeredBool_, id) ||
      contains(registered_data_.registeredMarkers_, id))
  {
#if !defined(NDEBUG)
    throw ExistingVariableException(id, this);
#else
    ROS_ERROR_STREAM("Registering an existing variable: " << id);
#endif
  }
  else
  {
    ROS_DEBUG_STREAM("Registered double: " << id);
    std::tuple<const std::string, const double *, double> p(id, variable, 0.);
    registered_data_.registeredDouble_.push_back(p);
    registeded_ids.push_back(id);
  }
  unlock();
}

void DynamicIntrospection::registerVariable(const Eigen::Vector3d *variable,
                                            const std::string &id,
                                            std::vector<std::string> &registeded_ids)
{
  registerVariable(&variable->x(), id + "_X", registeded_ids);
  registerVariable(&variable->y(), id + "_Y", registeded_ids);
  registerVariable(&variable->z(), id + "_Z", registeded_ids);
}

void DynamicIntrospection::registerVariable(const Eigen::Quaterniond *variable,
                                            const std::string &id,
                                            std::vector<std::string> &registeded_ids)
{
  registerVariable(&variable->coeffs().x(), id + "_QX", registeded_ids);
  registerVariable(&variable->coeffs().y(), id + "_QY", registeded_ids);
  registerVariable(&variable->coeffs().z(), id + "_QZ", registeded_ids);
  registerVariable(&variable->coeffs().w(), id + "_QW", registeded_ids);
}

void DynamicIntrospection::registerVariable(const bool *variable, const std::string &id,
                                            std::vector<std::string> &registeded_ids)
{
  lock();
  if (contains(registered_data_.registeredInt_, id) ||
      contains(registered_data_.registeredDouble_, id) ||
      contains(registered_data_.registeredBool_, id) ||
      contains(registered_data_.registeredMarkers_, id))
  {
#if !defined(NDEBUG)
    throw ExistingVariableException(id, this);
#else
    ROS_ERROR_STREAM("Registering an existing variable: " << id);
#endif
  }
  else
  {
    ROS_DEBUG_STREAM("Registered bool: " << id);
    std::tuple<const std::string, const bool *, bool> p(id, variable, false);
    registered_data_.registeredBool_.push_back(p);
    registeded_ids.push_back(id);
  }
  unlock();
}

void DynamicIntrospection::registerVariable(const visualization_msgs::MarkerArray *variable,
                                            const std::string &id,
                                            std::vector<std::string> &registeded_ids)
{
  lock();
  if (contains(registered_data_.registeredInt_, id) ||
      contains(registered_data_.registeredDouble_, id) ||
      contains(registered_data_.registeredBool_, id) ||
      contains(registered_data_.registeredMarkers_, id))
  {
#if !defined(NDEBUG)
    throw ExistingVariableException(id, this);
#else
    ROS_ERROR_STREAM("Registering an existing variable: " << id);
#endif
  }
  else
  {
    ROS_DEBUG_STREAM("Registered Marker: " << id);
    std::tuple<const std::string, const visualization_msgs::MarkerArray *, visualization_msgs::MarkerArray> p(
        id, variable, visualization_msgs::MarkerArray());
    registered_data_.registeredMarkers_.push_back(p);
    registeded_ids.push_back(id);
  }
  unlock();
}

/////////

void DynamicIntrospection::unRegisterVariable(const std::string &id)
{
  ROS_DEBUG_STREAM("unregistered variable: " << id);

  lock();
  if (!(contains(registered_data_.registeredInt_, id) ||
        !contains(registered_data_.registeredDouble_, id) ||
        !contains(registered_data_.registeredBool_, id) ||
        !contains(registered_data_.registeredMarkers_, id)))
  {
#if !defined(NDEBUG)
    throw DoesNotExistingVariableException(id, this);
#else
    ROS_ERROR_STREAM("Trying to delete a variable that is not registered: " << id);
#endif
  }
  else
  {
    int index_int = indexVector(registered_data_.registeredInt_, id);
    int index_double = indexVector(registered_data_.registeredDouble_, id);
    int index_bool = indexVector(registered_data_.registeredBool_, id);
    int index_markers = indexVector(registered_data_.registeredMarkers_, id);

    if (index_int >= 0)
    {
      auto it = registered_data_.registeredInt_.begin() + index_int;
      registered_data_.registeredInt_.erase(it);
    }
    else if (index_double >= 0)
    {
      auto it = registered_data_.registeredDouble_.begin() + index_double;
      registered_data_.registeredDouble_.erase(it);
    }
    else if (index_bool >= 0)
    {
      auto it = registered_data_.registeredBool_.begin() + index_bool;
      registered_data_.registeredBool_.erase(it);
    }
    else if (index_markers >= 0)
    {
      auto it = registered_data_.registeredMarkers_.begin() + index_markers;
      registered_data_.registeredMarkers_.erase(it);
    }
    else
    {
#if !defined(NDEBUG)
      throw DoesNotExistingVariableException(id, this);
#else
      ROS_ERROR_STREAM("Trying to delete a variable that is not registered: " << id);
#endif
    }
    ROS_DEBUG_STREAM("Deleting int: " << id);
  }
  unlock();

  ROS_DEBUG_STREAM("Succesfully deleted unregistered variable: " << id);
}
