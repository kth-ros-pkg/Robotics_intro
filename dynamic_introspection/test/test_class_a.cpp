#include "test_class_a.h"
#include <dynamic_introspection/dynamic_introspection.h>

TestClassA::TestClassA()
{
  cont_ = 0;
  cont_double_1_ = 0;
  cont_double_2_ = 0;

  REGISTER_VARIABLE(&cont_, "counterA", registered_ids_);
  REGISTER_VARIABLE(&cont_double_1_, "cont_double_1", registered_ids_);
  REGISTER_VARIABLE(&cont_double_2_, "cont_double_2", registered_ids_);
}


TestClassA::~TestClassA()
{
  UNREGISTER_VARIABLES(registered_ids_);
}

void TestClassA::update()
{
  std::cerr << "Update a: " << cont_ << std::endl;
  ++cont_;

  cont_double_1_ += 0.1;
  cont_double_2_ += 0.2;
}
