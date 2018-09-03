#include <dynamic_introspection/introspection_online_utils.h>

namespace dynamic_introspection
{
IntrospectionOnlineReader::IntrospectionOnlineReader(const std::string &topic)
{
  nh_.setCallbackQueue(&cb_queue_);
  spinner_.reset(new ros::AsyncSpinner(1, &cb_queue_));
  sub_ = nh_.subscribe(topic, 1000, &IntrospectionOnlineReader::introspectionCB, this);
}

IntrospectionOnlineReader::~IntrospectionOnlineReader()
{
}

void IntrospectionOnlineReader::start()
{
  spinner_->start();
}

void IntrospectionOnlineReader::stop()
{
  spinner_->stop();
}

void IntrospectionOnlineReader::introspectionCB(const IntrospectionMsgConstPtr &msg)
{
  addMsg(msg);
  read_messages_.push_back(*msg.get());
}

void IntrospectionOnlineReader::dumpRosBag(const std::string &bag_name)
{
  rosbag::Bag bag;
  bag.open(bag_name, rosbag::bagmode::Write);

  for (size_t i = 0; i < read_messages_.size(); ++i)
  {
    bag.write("/introspection_data", read_messages_[i].header.stamp, read_messages_[i]);
  }
  bag.close();
}
}
