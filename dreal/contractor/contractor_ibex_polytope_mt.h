#pragma once

#include <mutex>
#include <ostream>
#include <thread>
#include <vector>

#include <cuckoohash_map.hh>

#include "dreal/contractor/contractor_cell.h"
#include "dreal/contractor/contractor_ibex_polytope.h"
#include "dreal/contractor/contractor_status.h"
#include "dreal/solver/config.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {

class ContractorIbexPolytopeMt : public ContractorCell {
 public:
  /// Constructs IbexPolytopeMt contractor using @p f and @p vars.
  ContractorIbexPolytopeMt(std::vector<Formula> formulas, const Box& box,
                           const Config& config);

  /// Deleted copy constructor.
  ContractorIbexPolytopeMt(const ContractorIbexPolytopeMt&) = delete;

  /// Deleted move constructor.
  ContractorIbexPolytopeMt(ContractorIbexPolytopeMt&&) = delete;

  /// Deleted copy assign operator.
  ContractorIbexPolytopeMt& operator=(const ContractorIbexPolytopeMt&) = delete;

  /// Deleted move assign operator.
  ContractorIbexPolytopeMt& operator=(ContractorIbexPolytopeMt&&) = delete;

  /// Default destructor.
  ~ContractorIbexPolytopeMt() override;

  void Prune(ContractorStatus* cs) const override;
  std::ostream& display(std::ostream& os) const override;

 private:
  ContractorIbexPolytope* GetCtc(const Box& box) const;

  const std::vector<Formula> formulas_;
  const Config config_;

  // Lock to protect ctcs_.
  mutable std::mutex m_;
  // Save the pointers to ContractorIbexPolytope so that we can delete them in
  // destructor.
  mutable std::vector<ContractorIbexPolytope*> ctcs_;
  mutable cuckoohash_map<std::thread::id, ContractorIbexPolytope*> ctc_map_;
};

}  // namespace dreal
