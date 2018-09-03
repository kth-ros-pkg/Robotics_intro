#ifndef _TEST_CLASS_A_
#define _TEST_CLASS_A_

#include "test_class_abstract.h"
#include <vector>
#include <string>

class TestClassA : public TestClassBase
{
public:
  TestClassA();

  virtual ~TestClassA();

  void update();

private:
  double cont_double_1_;
  double cont_double_2_;
  int cont_;
  std::vector<std::string> registered_ids_;
};

#endif
