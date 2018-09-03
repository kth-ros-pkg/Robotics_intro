#ifndef _TEST_CLASS_B_
#define _TEST_CLASS_B_

#include "test_class_abstract.h"
#include <vector>
#include <string>

class TestClassB : public TestClassBase
{
public:
  TestClassB();

  virtual ~TestClassB();

  void update();

private:
  int cont_;
  std::vector<std::string> registered_ids_;
};

#endif
