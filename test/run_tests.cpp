/**
 * \file run_tests.cpp
 * \mainpage
 *    Runs all unit and integration tests for the ZeroErr RMD driver
 * \author
 *    Tobit Flatscher (github.com/2b-t)
*/

#include <gtest/gtest.h>


int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
