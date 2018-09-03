#include <ros/ros.h>
#include <dynamic_introspection/dynamic_introspection.h>
#include "test_class_a.h"
#include "test_class_b.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");

  // Setup debugging rosconsole
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  boost::shared_ptr<TestClassBase> tA(new TestClassA());
  boost::shared_ptr<TestClassBase> tB(new TestClassB());


  ROS_INFO("Spinning node");

  while (nh.ok())
  {
    tA->update();
    tB->update();

    std::cerr << "*********" << std::endl;
    PUBLISH_DEBUG_DATA_TOPIC
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
