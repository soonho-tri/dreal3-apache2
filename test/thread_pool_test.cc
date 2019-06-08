#include "ThreadPool/ThreadPool.h"

#include <iostream>

#include <gtest/gtest.h>

void Worker() { std::cerr << ThreadPool::get_thread_id(); }

void Doit() {
  constexpr int number_of_threads = 8;
  ThreadPool pool(number_of_threads);

  std::vector<std::future<void>> results;

  for (int i = 0; i < number_of_threads; ++i) {
    results.push_back(pool.enqueue(Worker));
  }
  Worker();

  for (int i = 0; i < number_of_threads; i++) {
    results[i].get();
  }
}

GTEST_TEST(THREAD_POOL_TEST, TEST) {
  Doit();
  Doit();
}
