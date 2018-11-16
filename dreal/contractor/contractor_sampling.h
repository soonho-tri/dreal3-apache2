#pragma once

#include <ostream>
#include <random>
#include <vector>

#include "dreal/contractor/contractor.h"
#include "dreal/contractor/contractor_cell.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {

/// Sampling contractor.
class ContractorSampling : public ContractorCell {
 public:
  /// Deleted default constructor.
  ContractorSampling() = delete;

  /// Constructs Sampling contractor using @p formulas and @p config.
  ContractorSampling(std::vector<Formula> formulas, const Box& box,
                     const Config& config);

  /// Deleted copy constructor.
  ContractorSampling(const ContractorSampling&) = delete;

  /// Deleted move constructor.
  ContractorSampling(ContractorSampling&&) = delete;

  /// Deleted copy assign operator.
  ContractorSampling& operator=(const ContractorSampling&) = delete;

  /// Deleted move assign operator.
  ContractorSampling& operator=(ContractorSampling&&) = delete;

  ~ContractorSampling() override = default;

  void Prune(ContractorStatus* cs) const override;
  std::ostream& display(std::ostream& os) const override;

 private:
  void SamplePoint(const Box& box, Environment* const env) const;

  const std::vector<Formula> formulas_;
  mutable std::mt19937_64 rg_;
};

}  // namespace dreal
