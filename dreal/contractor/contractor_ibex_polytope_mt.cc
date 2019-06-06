#include "dreal/contractor/contractor_ibex_polytope_mt.h"

#include <utility>

#include "ThreadPool/ThreadPool.h"

#include "dreal/util/assert.h"
#include "dreal/util/logging.h"
#include "dreal/util/timer.h"

using std::make_unique;
using std::ostream;
using std::vector;

namespace dreal {

ContractorIbexPolytopeMt::ContractorIbexPolytopeMt(vector<Formula> formulas,
                                                   const Box& box,
                                                   const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_POLYTOPE,
                     ibex::BitSet::empty(box.size()), config},
      formulas_{std::move(formulas)},
      config_{config},
      ctcs_(config_.number_of_jobs()) {
  DREAL_LOG_DEBUG("ContractorIbexPolytopeMt::ContractorIbexPolytopeMt");

  for (size_t i = 0; i < ctcs_.size(); ++i) {
    auto ctc_unique_ptr =
        make_unique<ContractorIbexPolytope>(formulas_, box, config_);
    ctcs_[i] = std::move(ctc_unique_ptr);
  }

  // Build input.
  mutable_input() = ctcs_[0]->input();
  is_dummy_ = ctcs_[0]->is_dummy();
}

ContractorIbexPolytope* ContractorIbexPolytopeMt::GetCtc() const {
  thread_local const int tid{ThreadPool::get_thread_id()};
  return ctcs_[tid].get();
}

void ContractorIbexPolytopeMt::Prune(ContractorStatus* cs) const {
  ContractorIbexPolytope* const ctc{GetCtc()};
  DREAL_ASSERT(ctc && !is_dummy_);
  return ctc->Prune(cs);
}

ostream& ContractorIbexPolytopeMt::display(ostream& os) const {
  os << "IbexPolytopeMt(";
  for (const Formula& f : formulas_) {
    os << f << ";";
  }
  return os << ")";
}

bool ContractorIbexPolytopeMt::is_dummy() const { return is_dummy_; }

}  // namespace dreal
