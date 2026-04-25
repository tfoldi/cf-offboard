// Single TU that emits doctest's main(). All other test_*.cpp files just
// include doctest.h and TEST_CASE freely.
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
