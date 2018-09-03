#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <dynamic_introspection/IntrospectionMsg.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

int main(int argc, char **argv) {

  // Check the number of parameters
  if (argc < 2) {
    // Tell the user how to run the program
    std::cerr << "Usage: bagName"<<std::endl;
    return 1;
  }
  // Print the user's name:
  std::string bagName = argv[1];

  std::cerr<<"Reading bag: "<<bagName<<std::endl;
  rosbag::Bag bag;
  bag.open(bagName, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string("data"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach(rosbag::MessageInstance const m, view)
  {

    std::string type = m.getDataType();
    std::string messageDeinitiaion  = m.getMessageDefinition();

    dynamic_introspection::IntrospectionMsg::ConstPtr s = m.instantiate<dynamic_introspection::IntrospectionMsg>();

    std::cerr<<"Number of Bool parameters: "<<s->bools.size()<<std::endl;
    for(size_t i=0; i < s->bools.size(); ++i){
      std::cerr<<"    "<<s->bools[i].name<<std::endl;
    }

    std::cerr<<"Number of Int parameters: "<<s->ints.size()<<std::endl;
    for(size_t i=0; i < s->ints.size(); ++i){
      std::cerr<<"    "<<s->ints[i].name<<std::endl;
    }

    std::cerr<<"Number of Double parameters: "<<s->doubles.size()<<std::endl;
    for(size_t i=0; i < s->doubles.size(); ++i){
      std::cerr<<"    "<<s->doubles[i].name<<std::endl;
    }

    break;
  }

  bag.close();

}
