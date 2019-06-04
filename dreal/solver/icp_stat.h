#pragma once

#include <atomic>

#include "dreal/util/stat.h"
#include "dreal/util/timer.h"

namespace dreal {

/// A class to show statistics information at destruction. We have a
/// static instance in Icp::CheckSat() to keep track of the numbers of
/// branching and pruning operations.
class IcpStat : public Stat {
 public:
  explicit IcpStat(const bool enabled, int id = 0)
      : Stat{enabled}, thread_id_{id} {}
  IcpStat(const IcpStat&) = delete;
  IcpStat(IcpStat&&) = delete;
  IcpStat& operator=(const IcpStat&) = delete;
  IcpStat& operator=(IcpStat&&) = delete;
  ~IcpStat() override;

  const int thread_id_;

  std::atomic<int> num_branch_{0};
  std::atomic<int> num_prune_{0};

  Timer timer_branch_;
  Timer timer_prune_;
  Timer timer_eval_;
};
}  // namespace dreal
