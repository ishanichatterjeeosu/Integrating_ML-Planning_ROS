#include "gtest/gtest.h"

// I shouldn't need to define this.  Why won't catkin link with
// libgtest_main.so? Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}