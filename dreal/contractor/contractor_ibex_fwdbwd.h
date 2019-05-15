#pragma once

#include <memory>
#include <mutex>
#include <ostream>
#include <thread>
#include <unordered_map>
#include <vector>

#include "./ibex.h"

#include <cuckoohash_map.hh>

#include "dreal/contractor/contractor.h"
#include "dreal/contractor/contractor_cell.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {

/// Contractor class wrapping IBEX's forward/backward contractor.
class ContractorIbexFwdbwd : public ContractorCell {
 public:
  /// Deleted default constructor.
  ContractorIbexFwdbwd() = delete;

  /// Constructs IbexFwdbwd contractor using @p f and @p box.
  ContractorIbexFwdbwd(Formula f, const Box& box, const Config& config);

  /// Deleted copy constructor.
  ContractorIbexFwdbwd(const ContractorIbexFwdbwd&) = delete;

  /// Deleted move constructor.
  ContractorIbexFwdbwd(ContractorIbexFwdbwd&&) = delete;

  /// Deleted copy assign operator.
  ContractorIbexFwdbwd& operator=(const ContractorIbexFwdbwd&) = delete;

  /// Deleted move assign operator.
  ContractorIbexFwdbwd& operator=(ContractorIbexFwdbwd&&) = delete;

  ~ContractorIbexFwdbwd();

  void Prune(ContractorStatus* cs) const override;

  /// Evaluates the constraint using the input @p box and returns the
  /// result.
  Box::Interval Evaluate(const Box& box) const;

  std::ostream& display(std::ostream& os) const override;

 private:
  ibex::CtcFwdBwd* GetCtc(const Box& box) const;

  const Formula f_;

  mutable std::mutex m_;

  mutable std::vector<IbexConverter*> ibex_converters_;

  mutable std::unordered_map<std::thread::id, const ibex::ExprCtr*>
      expr_ctr_map_;

  mutable cuckoohash_map<std::thread::id, ibex::CtcFwdBwd*> ctc_map_;
};

}  // namespace dreal
