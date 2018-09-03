#include "test_class_b.h"
#include <dynamic_introspection/dynamic_introspection.h>

TestClassB::TestClassB()
{
  cont_ = 0;
  REGISTER_VARIABLE(&cont_, "counterB", registered_ids_);
}

TestClassB::~TestClassB()
{
  UNREGISTER_VARIABLES(registered_ids_);
}

void TestClassB::update()
{
  std::cerr << "Update b: " << cont_ << std::endl;
  --cont_;
}
