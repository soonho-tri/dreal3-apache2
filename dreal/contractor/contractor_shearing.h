#pragma once

#include <memory>
#include <ostream>

#include "dreal/contractor/contractor.h"
#include "dreal/contractor/contractor_cell.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {

/// TODO(soonho): Fill this.
class ContractorShearing : public ContractorCell {
 public:
  /// Deleted default constructor.
  ContractorShearing() = delete;

  /// Constructs Shearing contractor using @p f and @p box.
  ContractorShearing(Formula f, const Box& box, const Config& config,
                     Contractor::ShearingMethod method, int n, double alpha);

  /// Deleted copy constructor.
  ContractorShearing(const ContractorShearing&) = delete;

  /// Deleted move constructor.
  ContractorShearing(ContractorShearing&&) = delete;

  /// Deleted copy assign operator.
  ContractorShearing& operator=(const ContractorShearing&) = delete;

  /// Deleted move assign operator.
  ContractorShearing& operator=(ContractorShearing&&) = delete;

  ~ContractorShearing() override = default;

  void Prune(ContractorStatus* cs) const override;

  std::ostream& display(std::ostream& os) const override;

 private:
  const Formula f_;
  RelationalOperator rop_;
  Expression e_;
  const Contractor::ShearingMethod method_;
  const int n_;
  const double alpha_;
};

}  // namespace dreal
