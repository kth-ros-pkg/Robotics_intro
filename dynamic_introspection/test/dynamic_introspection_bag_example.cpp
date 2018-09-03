#include <ros/ros.h>
#include <dynamic_introspection/dynamic_introspection.h>

int main(int argc, char **argv)
{
  // Setup debugging rosconsole
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");

  std::vector<std::string> registered_ids;

  bool bool_test = false;

  // DynamicIntrospection di(nh, "debug_test");

  REGISTER_VARIABLE(&bool_test, "bool_test", registered_ids);

  ROS_INFO("Spinning node");

  OPEN_BAG("test.bag")
  PUBLISH_DEBUG_DATA_BAG
  CLOSE_BAG


  return 0;
}
