#include <ros/ros.h>
#include <thread>
#include <mutex>

class ThreadedClass
{

public:

  ThreadedClass(){

    thread_ = std::thread(&ThreadedClass::run, this);
  }

  virtual ~ThreadedClass(){}

  virtual void run(){

    for(size_t i=0; i<10; ++i){
      std::cerr<<"hola thread: "<<i<<std::endl;
      ros::Duration(1.).sleep();
    }
  }

  virtual void stop(){}

  bool trylock()
  {
    if (msg_mutex_.try_lock())
    {
      if (turn_ == REALTIME)
      {
        return true;
      }
      else
      {
        msg_mutex_.unlock();
        return false;
      }
    }
    else
    {
      return false;
    }
  }



  void unlockAndPublish()
  {
    turn_ = NON_REALTIME;
    msg_mutex_.unlock();
    //00138 #ifdef NON_POLLING
    //00139     updated_cond_.notify_one();
    //00140 #endif
  }


protected:

  enum {REALTIME, NON_REALTIME};
  int turn_;  // Who's turn is it to use msg_?

  std::mutex msg_mutex_;  // Protects msg_
  std::thread thread_;

};


int main(int argc, char **argv){

  ros::init(argc, argv, "test");

  ros::NodeHandle nh("test");

  std::cerr<<"hola main"<<std::endl;
  ThreadedClass c;
  c.run();
  std::cerr<<"adios main"<<std::endl;

  ros::spin();
}
