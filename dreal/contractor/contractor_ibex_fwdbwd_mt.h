#pragma once

#include <memory>
#include <ostream>
#include <thread>

#include "dreal/contractor/contractor_cell.h"
#include "dreal/contractor/contractor_ibex_fwdbwd.h"
#include "dreal/contractor/contractor_status.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {

/// Multi-thread version of ContractorIbexFwdbwd contractor.
class ContractorIbexFwdbwdMt : public ContractorCell {
 public:
  /// Deleted default constructor.
  ContractorIbexFwdbwdMt() = delete;

  /// Constructs IbexFwdbwdMt contractor using @p f and @p box.
  ContractorIbexFwdbwdMt(Formula f, const Box& box, const Config& config);

  /// Deleted copy constructor.
  ContractorIbexFwdbwdMt(const ContractorIbexFwdbwdMt&) = delete;

  /// Deleted move constructor.
  ContractorIbexFwdbwdMt(ContractorIbexFwdbwdMt&&) = delete;

  /// Deleted copy assign operator.
  ContractorIbexFwdbwdMt& operator=(const ContractorIbexFwdbwdMt&) = delete;

  /// Deleted move assign operator.
  ContractorIbexFwdbwdMt& operator=(ContractorIbexFwdbwdMt&&) = delete;

  ~ContractorIbexFwdbwdMt() override = default;

  void Prune(ContractorStatus* cs) const override;

  std::ostream& display(std::ostream& os) const override;

 private:
  // Returns the corresponding contractor assigned to the current thread.
  ContractorIbexFwdbwd* GetCtc() const;

  // Returns the corresponding contractor assigned to the current
  // thread. If it does not exists, create one.
  ContractorIbexFwdbwd* GetCtcOrCreate(const Box& box) const;

  const Formula f_;
  const Config config_;

  // ctcs_ready_[i] is 1 if i-th contractor has been looked up before.
  mutable std::vector<int> ctcs_ready_;
  mutable std::vector<std::unique_ptr<ContractorIbexFwdbwd>> ctcs_;
};

}  // namespace dreal
