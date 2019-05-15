#include <memory>

#include <gtest/gtest.h>

#include <cuckoohash_map.hh>

GTEST_TEST(LIBCUCKOO_TEST, TEST1) {
  cuckoohash_map<int, std::unique_ptr<int>> map;
  map.insert(3, new int(3));
}
