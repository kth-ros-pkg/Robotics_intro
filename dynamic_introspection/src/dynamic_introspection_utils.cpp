#include <dynamic_introspection/dynamic_introspection_utils.h>
#include <rosbag/view.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace dynamic_introspection
{
// Returns if the element exists in the map and if it does it returns the element
template <class Key, class Value, class Comparator, class Alloc>
bool getMapValue(const std::map<Key, Value, Comparator, Alloc> &my_map, Key key, Value &out)
{
  typename std::map<Key, Value, Comparator, Alloc>::const_iterator it = my_map.find(key);
  if (it != my_map.end())
  {
    out = it->second;
    return true;
  }
  return false;
}

struct DoesNotExistingVariableExceptionUtils : public std::runtime_error
{
  DoesNotExistingVariableExceptionUtils(const unsigned index, const std::string &name,
                                        IntrospectionBagReader *br)
    : index_(index), std::runtime_error(""), br_(br), variable_name_(name)
  {
  }

  const char *what() const throw()
  {
    std::stringstream ss;
    ss << "Variable does not exist: " << variable_name_ << std::endl;
    ss << "Existing variables: " << std::endl;
    for (auto it = br_->doubleNameMap_[index_].begin();
         it != br_->doubleNameMap_[index_].end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    /*
    for (auto it = br_->intNameMap_[index_].begin(); it != br_->intNameMap_[index_].end();
    ++it)
    {
      ss << "   " << it->first << std::endl;
    }

    for (auto it = br_->boolNameMap_[index_].begin(); it !=
    br_->boolNameMap_[index_].end(); ++it)
    {
      ss << "   " << it->first << std::endl;
    }
    */

    ROS_ERROR_STREAM(ss.str());
    return ss.str().c_str();
  }

  unsigned int index_;
  std::string variable_name_;
  IntrospectionBagReader *br_;
};

IntrospectionBagReader::IntrospectionBagReader() : counter_(0), first_msg_(true)
{
}

IntrospectionBagReader::~IntrospectionBagReader()
{
}

IntrospectionBagReader::IntrospectionBagReader(const std::string &packageName,
                                               const std::string &bagFileName,
                                               const std::string introspection_topic_name)
  : introspection_topic_name_(introspection_topic_name), counter_(0), first_msg_(true)
{
  ROS_INFO_STREAM("Reading bag: " << bagFileName);

  rosbag::Bag bag;
  bag.open(bagFileName, rosbag::bagmode::Read);

  readBag(bag);
}

IntrospectionBagReader::IntrospectionBagReader(const std::string &bagFileName,
                                               const std::string introspection_topic_name)
  : introspection_topic_name_(introspection_topic_name), counter_(0), first_msg_(true)
{
  ROS_INFO_STREAM("Reading bag: " << bagFileName);

  rosbag::Bag bag;
  bag.open(bagFileName, rosbag::bagmode::Read);

  readBag(bag);
}

void IntrospectionBagReader::readBag(rosbag::Bag &bag)
{
  std::vector<std::string> topics;
  topics.push_back(std::string(introspection_topic_name_));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  unsigned int n_messages_ = view.size();

  ROS_INFO_STREAM("Number of messages: " << n_messages_);

  //  boolNameMap_.reserve(n_messages_);
  //  intNameMap_.reserve(n_messages_);
  //  doubleNameMap_.reserve(n_messages_);

  foreach (rosbag::MessageInstance const m, view)
  {
    dynamic_introspection::IntrospectionMsg::ConstPtr s =
        m.instantiate<dynamic_introspection::IntrospectionMsg>();

    addMsg(s);

    ROS_INFO_STREAM_THROTTLE(1.0,
                             "Reading percentage of data: "
                                 << ((double)counter_ / (double)n_messages_) * 100. << " %");
  }

  ROS_INFO_STREAM("Finished reading bag");
}

void IntrospectionBagReader::addMsg(const dynamic_introspection::IntrospectionMsg::ConstPtr s)
{
  if (first_msg_)
  {
    boolValues_.resize(s->bools.size());
    intValues_.resize(s->ints.size());
    doubleValues_.resize(s->doubles.size());

    first_msg_ = false;
  }

  boolNameMap_.push_back(std::map<std::string, int>());
  doubleNameMap_.push_back(std::map<std::string, int>());
  intNameMap_.push_back(std::map<std::string, int>());

  for (size_t i = 0; i < s->bools.size(); ++i)
  {
    // boolValues_[i].reserve(n_messages_);
    boolNameMap_[counter_][s->bools[i].name] = i;
  }
  for (size_t i = 0; i < s->ints.size(); ++i)
  {
    // intValues_[i].reserve(n_messages_);
    intNameMap_[counter_][s->ints[i].name] = i;
  }

  for (size_t i = 0; i < s->doubles.size(); ++i)
  {
    // doubleValues_[i].reserve(n_messages_);
    doubleNameMap_[counter_][s->doubles[i].name] = i;
  }

  for (size_t i = 0; i < s->bools.size(); ++i)
  {
    boolValues_[i].push_back(s->bools[i].value);
  }

  for (size_t i = 0; i < s->ints.size(); ++i)
  {
    intValues_[i].push_back(s->ints[i].value);
  }

  for (size_t i = 0; i < s->doubles.size(); ++i)
  {
    doubleValues_[i].push_back(s->doubles[i].value);
  }

  ++counter_;
}

unsigned int IntrospectionBagReader::getNumberMessages()
{
  return counter_;
}

void IntrospectionBagReader::getVariable(const std::string &variableId,
                                         std::vector<bool> &value, const bool throw_not_existing)
{
  assert(value.size() == 0);

  value.reserve(boolNameMap_.size());

  for (size_t i = 0; i < boolNameMap_.size(); ++i)
  {
    int index = -1;
    if (!getMapValue(boolNameMap_[i], variableId, index) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId, this);
    }
    if (index >= 0)
    {
      value.push_back(boolValues_[index][i]);
    }
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId,
                                         std::vector<double> &value, const bool throw_not_existing)
{
  assert(value.size() == 0);

  value.reserve(doubleNameMap_.size());

  for (size_t i = 0; i < doubleNameMap_.size(); ++i)
  {
    int index = -1;
    if (!getMapValue(doubleNameMap_[i], variableId, index) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId, this);
    }
    if (index >= 0)
    {
      value.push_back(doubleValues_[index][i]);
    }
  }
}

void IntrospectionBagReader::getVariable(const std::string &variableId,
                                         std::vector<Eigen::Vector3d> &value,
                                         const bool throw_not_existing)
{
  assert(value.size() == 0);

  std::vector<std::string> ids;
  ids.push_back(variableId + "_X");
  ids.push_back(variableId + "_Y");
  ids.push_back(variableId + "_Z");
  getVariable(ids, value, throw_not_existing);
}

void IntrospectionBagReader::getVariable(const std::vector<std::string> &variableId,
                                         std::vector<Eigen::Vector3d> &value,
                                         const bool throw_not_existing)
{
  assert(value.size() == 0);
  value.reserve(doubleNameMap_.size());

  assert(variableId.size() == 3);

  for (size_t i = 0; i < doubleNameMap_.size(); ++i)
  {
    int index1 = -1;
    if (!getMapValue(doubleNameMap_[i], variableId[0], index1) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId[0], this);
    }
    int index2 = -1;
    if (!getMapValue(doubleNameMap_[i], variableId[1], index2) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId[1], this);
    }
    int index3 = -1;
    if (!getMapValue(doubleNameMap_[i], variableId[2], index3) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variableId[2], this);
    }

    if ((index1 >= 0) && (index2 >= 0) && (index3 >= 0))
    {
      Eigen::Vector3d v(doubleValues_[index1][i], doubleValues_[index2][i],
                        doubleValues_[index3][i]);
      //      nanDetectedEigen(v);
      for (size_t i = 0; i < 3; ++i)
      {
        if (std::isnan(v[i]) || std::isinf(v[i]))
        {
          ROS_WARN_STREAM("found nan while parsing variable: " << variableId[i]);
          v[i] = 0.;
        }
      }
      value.push_back(v);
    }
  }
}


void IntrospectionBagReader::getVariable(const std::string &variable_id,
                                         std::vector<Eigen::Quaterniond> &value,
                                         const bool throw_not_existing)
{
  assert(value.size() == 0);
  value.reserve(doubleNameMap_.size());

  for (size_t i = 0; i < doubleNameMap_.size(); ++i)
  {
    int index1 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QX", index1) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QX", this);
    }
    int index2 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QY", index2) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QY", this);
    }
    int index3 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QZ", index3) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QZ", this);
    }

    int index4 = -1;
    if (!getMapValue(doubleNameMap_[i], variable_id + "_QW", index4) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(i, variable_id + "_QW", this);
    }

    if ((index1 >= 0) && (index2 >= 0) && (index3 >= 0) && (index4 >= 0))
    {
      value.push_back(Eigen::Quaterniond(doubleValues_[index4][i], doubleValues_[index1][i],
                                         doubleValues_[index2][i], doubleValues_[index3][i]));
    }
  }
}

/*
void IntrospectionBagReader::getVariable(const std::vector<std::string> &names,
                                         std::vector<Eigen::VectorXd> &value,
                                         const bool throw_not_existing)
{
  Eigen::VectorXd temp(names.size());
  value.reserve(n_messages_);
  for (size_t i = 0; i < names.size(); ++i)
  {
    assert(n_messages_ == value[i].size());

    int index;
    if (!getMapValue(doubleNameMap_, names[i], index) && throw_not_existing)
    {
      throw DoesNotExistingVariableExceptionUtils(names[i], this);
    }

    for (size_t j = 0; j < n_messages_; ++j)
    {
      temp(j) = doubleValues_[i][j];
    }

    value.push_back(temp);
  }
}
*/
}
