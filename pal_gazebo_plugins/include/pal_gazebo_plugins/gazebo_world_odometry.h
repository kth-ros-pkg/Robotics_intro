#ifndef GAZEBO_WORLD_ODOMETRY_H
#define GAZEBO_WORLD_ODOMETRY_H

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace gazebo {

  class Entity;

  class GazeboWorldOdometry : public ModelPlugin{

  public:
    GazeboWorldOdometry();
    ~GazeboWorldOdometry();
    void DeferredLoad();
    void Load(physics::ModelPtr parent_model, sdf::ElementPtr sdf);

  protected:
    virtual void UpdateChild();

  private:
    void RosQueueThread();

    /// \brief for setting ROS name space
    std::string robot_namespace_;
    std::string topic_name_;
    std::string frame_name_;

    ros::NodeHandle* rosNode_;
    double      update_rate_;

    gazebo::physics::WorldPtr world_;
    ros::Publisher floatingBasePub_;
    gazebo::physics::LinkPtr link;

    event::ConnectionPtr update_connection_;
    boost::thread deferredLoadThread;
    ros::CallbackQueue rosQueue;
    boost::thread callbackQueeuThread;

    // Mutex
    private: boost::mutex mutex_;

  };

}

#endif // GAZEBO_HARNESS_H
