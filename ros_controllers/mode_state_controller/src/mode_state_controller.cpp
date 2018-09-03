#include <algorithm>
#include <cstddef>

#include <mode_state_controller/mode_state_controller.h>

namespace mode_state_controller
{
bool ModeStateController::init(hardware_interface::JointModeInterface* hw,
                               ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  // get all joint names from the hardware interface
  const std::vector<std::string>& actuator_names = hw->getNames();
  num_hw_joints_ = actuator_names.size();
  for (unsigned i = 0; i < num_hw_joints_; i++)
    ROS_DEBUG("Got actuator %s", actuator_names[i].c_str());

  // get publishing period
  if (!controller_nh.getParam("publish_rate", publish_rate_))
  {
    ROS_ERROR("Parameter 'publish_rate' not set");
    return false;
  }

  // realtime publisher
  realtime_pub_.reset(new realtime_tools::RealtimePublisher<mode_state_controller::ModeState>(
      root_nh, "actuator_mode_state", 4));

  // get joints and allocate message
  for (unsigned i = 0; i < num_hw_joints_; i++)
  {
    try
    {
      actuator_state_.push_back(hw->getHandle(actuator_names[i]));
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      std::vector<std::string> resource_names = hw->getNames();
      std::stringstream ss;
      ss << e.what() << std::endl << "Available resources: " << std::endl;
      for (size_t i = 0; i < resource_names.size(); ++i)
      {
        ss << resource_names[i] << std::endl;
      }
      throw hardware_interface::HardwareInterfaceException(ss.str());
    }
    realtime_pub_->msg_.name.push_back(actuator_names[i]);
    realtime_pub_->msg_.mode.push_back(mode_state_controller::ModeState::NOMODE);
  }

  return true;
}

void ModeStateController::starting(const ros::Time& time)
{
  // initialize time
  last_publish_time_ = time;
}

void ModeStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
{
  // limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
  {
    // try to publish
    if (realtime_pub_->trylock())
    {
      // we're actually publishing, so increment time
      last_publish_time_ = last_publish_time_ + ros::Duration(1.0 / publish_rate_);

      // populate joint state message:
      // - fill only joints that are present in the JointStateInterface, i.e. indices [0,
      // num_hw_joints_)
      // - leave unchanged extra joints, which have static values, i.e. indices from
      // num_hw_joints_ onwards
      realtime_pub_->msg_.header.stamp = time;
      for (unsigned i = 0; i < num_hw_joints_; i++)
      {
        realtime_pub_->msg_.mode[i] = actuator_state_[i].getMode();
      }
      realtime_pub_->unlockAndPublish();
    }
  }
}

void ModeStateController::stopping(const ros::Time& /*time*/)
{
}
}

PLUGINLIB_EXPORT_CLASS(mode_state_controller::ModeStateController, controller_interface::ControllerBase)
