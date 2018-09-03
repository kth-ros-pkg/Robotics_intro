///////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2014, 2015 PAL Robotics S.L.

// All rights reserved.

//////////////////////////////////////////////////////////////////////////////

// Author Hilario Tomé

#ifndef _DYNAMIC_INTROSPECTION_
#define _DYNAMIC_INTROSPECTION_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <realtime_tools/realtime_publisher.h>
#include <Eigen/Dense>

/**
 * @brief The DynamicIntrospection class allows to do dynamic instrospection of different
 * c++ types through ros topics and rosbag
 */

struct DynamicIntrospectionData
{
  // Registered variables
  std::vector<std::tuple<std::string, const int *, int> > registeredInt_;
  std::vector<std::tuple<std::string, const double *, double> > registeredDouble_;
  std::vector<std::tuple<std::string, const bool *, bool> > registeredBool_;
  std::vector<std::tuple<std::string, const visualization_msgs::MarkerArray *, visualization_msgs::MarkerArray> > registeredMarkers_;

  void copy()
  {
    for (size_t i = 0; i < registeredInt_.size(); ++i)
    {
      std::get<2>(registeredInt_[i]) = *std::get<1>(registeredInt_[i]);
    }

    for (size_t i = 0; i < registeredDouble_.size(); ++i)
    {
      std::get<2>(registeredDouble_[i]) = *std::get<1>(registeredDouble_[i]);
    }

    for (size_t i = 0; i < registeredBool_.size(); ++i)
    {
      std::get<2>(registeredBool_[i]) = *std::get<1>(registeredBool_[i]);
    }

    for (size_t i = 0; i < registeredMarkers_.size(); ++i)
    {
      std::get<2>(registeredMarkers_[i]) = *std::get<1>(registeredMarkers_[i]);
    }
  }
};

class DynamicIntrospection
{
public:
  static DynamicIntrospection *Instance();

  virtual ~DynamicIntrospection();

  void registerVariable(const int *variable, const std::string &id,
                        std::vector<std::string> &registeded_ids);
  void registerVariable(const double *variable, const std::string &id,
                        std::vector<std::string> &registeded_ids);
  void registerVariable(const Eigen::Vector3d *variable, const std::string &id,
                        std::vector<std::string> &registeded_ids);
  void registerVariable(const Eigen::Quaterniond *variable, const std::string &id,
                        std::vector<std::string> &registeded_ids);
  void registerVariable(const bool *variable, const std::string &id,
                        std::vector<std::string> &registeded_ids);
  void registerVariable(const visualization_msgs::MarkerArray *variable,
                        const std::string &id, std::vector<std::string> &registeded_ids);

  void unRegisterVariable(const std::string &id);

  void setOutputTopic(const std::string &outputTopic);

  void generateMessage();

  bool trylock()
  {
    if (msg_mutex_.try_lock())
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void lock()
  {
    msg_mutex_.lock();
  }

  void unlock()
  {
    msg_mutex_.unlock();
  }

  void unlockAndPublish()
  {
    msg_mutex_.unlock();
    updated_cond_.notify_one();
  }

  void publishDataTopicRT();

  void publishDataBag();

  void closeBag();

  void openBag(std::string fileName);

  const DynamicIntrospectionData *getDataPtr() const
  {
    return &registered_data_;
  }

private:
  static DynamicIntrospection *m_pInstance;

  DynamicIntrospection();

  void publishDataTopic();

  bool openedBag_;
  bool configured_;

  ros::NodeHandle node_handle_;
  ros::Publisher introspectionPub_;

  rosbag::Bag bag_;

  DynamicIntrospectionData registered_data_;

  dynamic_introspection::IntrospectionMsg introspectionMessage_;

  boost::thread thread_;
  boost::mutex msg_mutex_;  // Protects msg_

  boost::mutex updated_cond__mutex_;
  boost::condition_variable updated_cond_;
};

typedef boost::shared_ptr<DynamicIntrospection> DynamicIntrospectionPtr;

#define REGISTER_VARIABLE(VARIABLE, ID, REGISTERED_VARIABLES_VECTOR)                     \
  DynamicIntrospection::Instance()->registerVariable(VARIABLE, ID, REGISTERED_VARIABLES_VECTOR);

#define UNREGISTER_VARIABLES(REGISTERED_VARIABLES_VECTOR)                                 \
  for (size_t i = 0; i < REGISTERED_VARIABLES_VECTOR.size(); ++i)                         \
  {                                                                                       \
    DynamicIntrospection::Instance()->unRegisterVariable(REGISTERED_VARIABLES_VECTOR[i]); \
  }                                                                                       \
  REGISTERED_VARIABLES_VECTOR.clear();

#define OPEN_BAG(BAG_NAME) DynamicIntrospection::Instance()->openBag(BAG_NAME);

#define PUBLISH_DEBUG_DATA_BAG DynamicIntrospection::Instance()->publishDataBag();

#define PUBLISH_DEBUG_DATA_TOPIC DynamicIntrospection::Instance()->publishDataTopicRT();

#define CLOSE_BAG DynamicIntrospection::Instance()->closeBag();

#define CONFIGURE_OUTPUT_TOPIC(TOPIC_NAME)                                               \
  DynamicIntrospection::Instance()->setOutputTopic(TOPIC_NAME);


#endif
