#ifndef _MODE_STATE_CONTROLLER_
#define _MODE_STATE_CONTROLLER_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_publisher.h>
#include <boost/shared_ptr.hpp>
#include <hardware_interface/joint_mode_interface.h>
#include <mode_state_controller/ModeState.h>

namespace mode_state_controller
{

  class ModeStateController: public controller_interface::Controller<hardware_interface::JointModeInterface>
  {

  public:

    ModeStateController() : publish_rate_(0.0) {}

    virtual bool init(hardware_interface::JointModeInterface* hw,
                      ros::NodeHandle&                         root_nh,
                      ros::NodeHandle&                         controller_nh);
    virtual void starting(const ros::Time& time);
    virtual void update(const ros::Time& time, const ros::Duration& /*period*/);
    virtual void stopping(const ros::Time& /*time*/);

  private:

    std::vector<hardware_interface::JointModeHandle> actuator_state_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<mode_state_controller::ModeState> > realtime_pub_;
    ros::Time last_publish_time_;
    double publish_rate_;
    unsigned int num_hw_joints_; ///< Number of joints present in the JointStateInterface, excluding extra joints
  };

}

#endif

