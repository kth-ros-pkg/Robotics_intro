#include <algorithm>
#include <cstddef>

#include <joint_torque_sensor_state_controller/joint_torque_sensor_state_controller.h>
#include <XmlRpc.h>

namespace joint_torque_sensor_state_controller
{

template <class Key, class Value, class Comparator, class Alloc>
bool contains(const std::map<Key, Value, Comparator, Alloc>& my_map, Key key)
{
  typename std::map<Key, Value, Comparator, Alloc>::const_iterator it = my_map.find(key);
  if (it != my_map.end() )
  {
    return true;
  }
  return false;
}

template <class T>
int indexVector(const std::vector<T> &v, const T &e){
  auto it = std::find(v.begin(), v.end(), e);
  if (it == v.end())
  {
    // name not in vector
    return -1;
  } else
  {
    return std::distance(v.begin(), it);
  }
  return -1;
}

bool JointStateTorqueSensorController::parseJointOffsets(ros::NodeHandle &controller_nh){

  using namespace XmlRpc;
  XmlRpcValue default_conf;
  if (controller_nh.getParam("joint_offsets", default_conf))
  {
    try{
      ROS_ERROR_STREAM("Number of joint offsets : "<<default_conf.size());
      for(XmlRpc::XmlRpcValue::ValueStruct::iterator it = default_conf.begin(); it != default_conf.end(); ++it ){
        std::string joint_name = static_cast<std::string>(it->first);
        double joint_offset = static_cast<double>(it->second);

        int index = indexVector(joint_names_, joint_name);
        if(index < 0){
          std::stringstream ss;
          ss<<"joint: "<<joint_name<< " not in robot model, available joints: "<<std::endl;
          for(size_t i=0; i<joint_names_.size(); ++i){
            ss<<"     "<<joint_names_[i]<<std::endl;
          }
          ROS_ERROR_STREAM(ss.str());
          return false;
        }
        ROS_ERROR_STREAM("Parsed joint offset conf "<<joint_name<<" :"<<joint_offset);
        joint_offsets_[joint_name]  = joint_offset;

      }
    }
    catch (XmlRpc::XmlRpcException exception) {
      ROS_ERROR_STREAM("Exception code: " + exception.getCode());
      ROS_ERROR_STREAM("Exception message: " + exception.getMessage());
      return false;
    }
  }

  return true;
}

bool JointStateTorqueSensorController::init(hardware_interface::JointStateInterface* hw,
                                            ros::NodeHandle&                         root_nh,
                                            ros::NodeHandle&                         controller_nh)
{
  // get all joint names from the hardware interface
  joint_names_ = hw->getNames();
  num_hw_joints_ = joint_names_.size();
  for (unsigned i=0; i<num_hw_joints_; i++)
    ROS_DEBUG("Got joint %s", joint_names_[i].c_str());

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_)){
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  std::string output_topic;
  if (!controller_nh.getParam("output_topic", output_topic)){
    ROS_ERROR("Parameter 'output_topic'' not specified");
    return false;
  }

  use_relative_for_nans_ = false;
  controller_nh.getParam("use_relative_for_nans", use_relative_for_nans_);
  if(use_relative_for_nans_){
    ROS_ERROR_STREAM("using relative encoders for nans");
  }
  else{
    ROS_ERROR_STREAM("NOT using relative encoders for nans");
  }

  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(root_nh, output_topic, 4));

  // get joints and allocate message
  for (unsigned i=0; i<num_hw_joints_; i++){
    joint_state_.push_back(hw->getHandle(joint_names_[i]));
    realtime_pub_->msg_.name.push_back(joint_names_[i]);
    realtime_pub_->msg_.position.push_back(0.0);
    realtime_pub_->msg_.velocity.push_back(0.0);
    realtime_pub_->msg_.effort.push_back(0.0);
  }
  addExtraJoints(controller_nh, realtime_pub_->msg_);

  if(!parseJointOffsets(controller_nh)){
    return false;
  }

  return true;
}

void JointStateTorqueSensorController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
}

void JointStateTorqueSensorController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

    // try to publish
    if (realtime_pub_->trylock()){
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

      // populate joint state message:
      // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
      // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
      realtime_pub_->msg_.header.stamp = time;
      for (unsigned i=0; i<num_hw_joints_; i++){
        realtime_pub_->msg_.position[i] = joint_state_[i].getAbsolutePosition();
        realtime_pub_->msg_.velocity[i] = joint_state_[i].getVelocity();
        realtime_pub_->msg_.effort[i] = joint_state_[i].getTorqueSensor();
        if(use_relative_for_nans_ && std::isnan(realtime_pub_->msg_.position[i])){
          realtime_pub_->msg_.position[i] = joint_state_[i].getPosition();
        }

        // Check if offset needs to be applied
        if(contains(joint_offsets_, joint_names_[i])){
          realtime_pub_->msg_.position[i] += joint_offsets_[joint_names_[i]];
        }
      }
      realtime_pub_->unlockAndPublish();
    }
  }
}

void JointStateTorqueSensorController::stopping(const ros::Time& /*time*/)
{}

void JointStateTorqueSensorController::addExtraJoints(const ros::NodeHandle& nh, sensor_msgs::JointState& msg)
{

  // Preconditions
  XmlRpc::XmlRpcValue list;
  if (!nh.getParam("extra_joints", list))
  {
    ROS_DEBUG("No extra joints specification found.");
    return;
  }

  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("Extra joints specification is not an array. Ignoring.");
    return;
  }

  for(std::size_t i = 0; i < list.size(); ++i)
  {
    if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR_STREAM("Extra joint specification is not a struct, but rather '" << list[i].getType() <<
                       "'. Ignoring.");
      continue;
    }

    if (!list[i].hasMember("name"))
    {
      ROS_ERROR_STREAM("Extra joint does not specify name. Ignoring.");
      continue;
    }

    const std::string name = list[i]["name"];
    if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
    {
      ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
      continue;
    }

    const bool has_pos = list[i].hasMember("position");
    const bool has_vel = list[i].hasMember("velocity");
    const bool has_eff = list[i].hasMember("effort");

    const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
    if (has_pos && list[i]["position"].getType() != typeDouble)
    {
      ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default position. Ignoring.");
      continue;
    }
    if (has_vel && list[i]["velocity"].getType() != typeDouble)
    {
      ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default velocity. Ignoring.");
      continue;
    }
    if (has_eff && list[i]["effort"].getType() != typeDouble)
    {
      ROS_ERROR_STREAM("Extra joint '" << name << "' does not specify a valid default effort. Ignoring.");
      continue;
    }

    // State of extra joint
    const double pos = has_pos ? static_cast<double>(list[i]["position"]) : 0.0;
    const double vel = has_vel ? static_cast<double>(list[i]["velocity"]) : 0.0;
    const double eff = has_eff ? static_cast<double>(list[i]["effort"])   : 0.0;

    // Add extra joints to message
    msg.name.push_back(name);
    msg.position.push_back(pos);
    msg.velocity.push_back(vel);
    msg.effort.push_back(eff);
  }
}

}

PLUGINLIB_EXPORT_CLASS(joint_torque_sensor_state_controller::JointStateTorqueSensorController, controller_interface::ControllerBase)
