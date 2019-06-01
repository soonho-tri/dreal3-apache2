#include "ThreadPool/ThreadPool.h"

#include <iostream>

#include <gtest/gtest.h>

// From https://github.com/progschj/ThreadPool/blob/master/README.md
GTEST_TEST(THREAD_POOL_TEST, TEST) {
  // create thread pool with 2 worker threads
  ThreadPool pool(2);

  // enqueue and store future
  auto result = pool.enqueue([](int answer) { return answer; }, 42);

  // get result from future
  EXPECT_EQ(result.get(), 42);
}
