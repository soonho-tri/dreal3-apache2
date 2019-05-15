#pragma once

#include <thread>
#include <utility>

namespace dreal {

class JoiningThread {
 public:
  template <typename F, typename... Args>
  explicit JoiningThread(F&& f, Args&&... args)
      : t_(std::forward<F>(f), std::forward<Args>(args)...) {}

  JoiningThread(JoiningThread&& jt) noexcept : t_{std::move(jt.t_)} {}

  ~JoiningThread() {
    if (t_.joinable()) {
      t_.join();
    }
  }

  JoiningThread(JoiningThread&) = delete;
  JoiningThread& operator=(JoiningThread const&) = delete;

 private:
  std::thread t_;
};

}  // namespace dreal
