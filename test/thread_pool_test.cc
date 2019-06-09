#include "ThreadPool/ThreadPool.h"

#include <iostream>

#include <gtest/gtest.h>

void Worker(ThreadPool* p) { std::cerr << p->get_tid(); }

void Doit() {
  constexpr int number_of_threads = 8;
  ThreadPool pool(number_of_threads);

  std::vector<std::future<void>> results;

  for (int i = 0; i < number_of_threads; ++i) {
    results.push_back(pool.enqueue(Worker, &pool));
  }
  Worker(&pool);

  for (int i = 0; i < number_of_threads; i++) {
    results[i].get();
  }
}

GTEST_TEST(THREAD_POOL_TEST, TEST) {
  Doit();
  Doit();
}
