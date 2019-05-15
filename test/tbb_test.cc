#include <cstdlib>
#include <iostream>

#include <gtest/gtest.h>

#include <tbb/task_group.h>

int fib(int n) {
  if (n < 2) {
    return n;
  } else {
    int x, y;
    tbb::task_group g;
    g.run([&] { x = fib(n - 1); });  // spawn a task
    g.run([&] { y = fib(n - 2); });  // spawn another task
    g.wait();                        // wait for both tasks to complete
    return x + y;
  }
}

GTEST_TEST(TBB_TEST, TEST1) { std::cout << fib(10) << "\n"; }
