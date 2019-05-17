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

  ~ContractorIbexFwdbwd() override;

  void Prune(ContractorStatus* cs) const override;

  /// Evaluates the constraint using the input @p box and returns the
  /// result.
  Box::Interval Evaluate(const Box& box) const;

  std::ostream& display(std::ostream& os) const override;

 private:
  const Formula f_;
  IbexConverter ibex_converter_;
  std::unique_ptr<const ibex::ExprCtr> expr_ctr_;
  std::unique_ptr<ibex::CtcFwdBwd> ctc_;
};

/// Contractor class wrapping IBEX's forward/backward contractor.
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

  ~ContractorIbexFwdbwdMt() override;

  void Prune(ContractorStatus* cs) const override;

  /// Evaluates the constraint using the input @p box and returns the
  /// result.
  Box::Interval Evaluate(const Box& box) const;

  std::ostream& display(std::ostream& os) const override;

 private:
  // TODO(soonho): Document.
  ContractorIbexFwdbwd* GetCtc() const;

  // TODO(soonho): Document.
  ContractorIbexFwdbwd* GetCtc(const Box& box) const;

  const Formula f_;
  const Config config_;

  // Lock to protect ibex_converters_ and expr_ctr_map_.
  mutable std::mutex m_;
  // Save the pointers to ContractorIbexFwdbwd so that we can delete them in
  // destructor.
  mutable std::vector<ContractorIbexFwdbwd*> ctcs_;
  mutable cuckoohash_map<std::thread::id, ContractorIbexFwdbwd*> ctc_map_;
};

}  // namespace dreal
