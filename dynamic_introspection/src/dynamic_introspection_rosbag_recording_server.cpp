#include <ros/ros.h>
#include <ros/package.h>

class DynamicIntrospectionRecorderServer{


public:

  DynamicIntrospectionRecorderServer(ros::NodeHandle &nh, const std::string &folderToPlaceBags,
                                     const std::string &introSpectionTopic):
    nh_(nh),
    folderToPlaceBags_(folderToPlaceBags),
    introSpectionTopic_(introSpectionTopic){


    //sub_ = nh_.subscribe(introSpectionTopic_, 1000, DynamicIntrospectionRecorderServer::callback, this, _1);
  }

  void callback(){

  }

  void update(){

  }


private:

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  std::string folderToPlaceBags_;
  std::string introSpectionTopic_;

};


int main(int argc, char **argv){

  // Set up ROS.
  ros::init(argc, argv, "biped_walking_dcm_controller");
  ros::NodeHandle nh;

  std::string introSpectionTopic = "/data";
  std::string folderToPlaceBags = "dynamic_introspection_recorded_bags";

  DynamicIntrospectionRecorderServer server(nh, folderToPlaceBags, introSpectionTopic);

  while(nh.ok()){

    ros::spinOnce();

    server.update();

    ros::Duration(0.0001).sleep();
  }

}
