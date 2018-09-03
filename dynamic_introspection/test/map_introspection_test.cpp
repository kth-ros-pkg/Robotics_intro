#include <ros/ros.h>
#include <dynamic_introspection/DynamicIntrospection.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "dynamic_introspection_test");

  ros::NodeHandle nh("dynamic_introspection_test");

  // Setup debugging rosconsole
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)){
    ros::console::notifyLoggerLevelsChanged();
  }

  Eigen::Vector3d vectorTest1;
  vectorTest1.setRandom();
  Eigen::Vector3d vectorTest2;
  vectorTest2.setRandom();
  REGISTER_VARIABLE(&vectorTest1, "vector1_test");
  REGISTER_VARIABLE(&vectorTest2, "vector2_test");

  Eigen::Map<const Eigen::Vector3d> mapTest1(vectorTest1.data());
  REGISTER_VARIABLE(&mapTest1, "map1_test");
  Eigen::Map<const Eigen::Vector3d> mapTest2(vectorTest2.data());
  REGISTER_VARIABLE(&mapTest2, "map2_test");

  ROS_INFO("Spinning node");

  while(nh.ok()){
    std::cerr<<"*********"<<std::endl;
    PUBLISH_DEBUG_DATA_TOPIC
    ros::spinOnce();
    ros::Duration(0.1).sleep();

    vectorTest1.setRandom();
    vectorTest2.setRandom();

  }
  return 0;
}
