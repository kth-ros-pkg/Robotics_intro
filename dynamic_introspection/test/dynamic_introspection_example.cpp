#include <ros/ros.h>
#include <dynamic_introspection/dynamic_introspection.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");

  // Setup debugging rosconsole
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    ros::console::notifyLoggerLevelsChanged();
  }

  Eigen::Vector3d test_vector = Eigen::Vector3d::Zero();
  bool bool_test = false;

  Eigen::Vector3d *test_vector_pointer = new Eigen::Vector3d(0,0,0);
  test_vector_pointer->x();


  std::vector<std::string> registered_ids;

  //DynamicIntrospection di(nh, "debug_test");

  REGISTER_VARIABLE(&bool_test, "bool_test", registered_ids);

  ROS_INFO("Spinning node");

  while(nh.ok()){
    std::cerr<<"*********"<<std::endl;
    PUBLISH_DEBUG_DATA_TOPIC
    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  return 0;
}
