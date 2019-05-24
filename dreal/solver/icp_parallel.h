#pragma once

#include <condition_variable>
#include <vector>

#include "dreal/contractor/contractor.h"
#include "dreal/contractor/contractor_status.h"
#include "dreal/solver/config.h"
#include "dreal/solver/formula_evaluator.h"
#include "dreal/solver/icp.h"

#include "dreal/util/joining_thread.h"

namespace dreal {

/// Class for ICP (Interval Constraint Propagation) algorithm.
class IcpParallel : public Icp {
 public:
  /// Constructs an IcpParallel based on @p config.
  explicit IcpParallel(const Config& config);

  bool CheckSat(const Contractor& contractor,
                const std::vector<FormulaEvaluator>& formula_evaluators,
                ContractorStatus* cs) override;

 private:
  std::vector<ContractorStatus> status_vector_;
  std::vector<JoiningThread> workers_;
  std::vector<int> ready_to_start_;
  std::vector<int> need_to_kill_;

  std::condition_variable cv_;
  std::mutex lock_;
};

}  // namespace dreal
