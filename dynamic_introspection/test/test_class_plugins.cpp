#include "test_class_a.h"
#include "test_class_b.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(TestClassA, TestClassBase);
PLUGINLIB_EXPORT_CLASS(TestClassB, TestClassBase);
