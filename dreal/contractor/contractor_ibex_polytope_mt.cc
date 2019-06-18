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
    : ContractorCell{Contractor::Kind::IBEX_POLYTOPE, config},
      formulas_{std::move(formulas)},
      config_{config},
      ctc_ready_(config_.number_of_jobs(), 0),
      ctcs_(ctc_ready_.size()) {
  DREAL_LOG_DEBUG("ContractorIbexPolytopeMt::ContractorIbexPolytopeMt");
}

const ibex::BitSet& ContractorIbexPolytopeMt::input() const {
  DREAL_ASSERT(ctc_ready_[0]);
  return ctcs_[0]->input();
}

ibex::BitSet& ContractorIbexPolytopeMt::mutable_input() {
  DREAL_ASSERT(ctc_ready_[0]);
  return ctcs_[0]->mutable_input();
}

ContractorIbexPolytope* ContractorIbexPolytopeMt::GetCtcOrCreate(
    const Box& box) const {
  thread_local const int tid{ThreadPool::get_thread_id()};
  if (ctc_ready_[tid]) {
    return ctcs_[tid].get();
  }
  auto ctc_unique_ptr =
      make_unique<ContractorIbexPolytope>(formulas_, box, config_);
  ContractorIbexPolytope* ctc = ctc_unique_ptr.get();
  DREAL_ASSERT(ctc);
  ctcs_[tid] = std::move(ctc_unique_ptr);
  ctc_ready_[tid] = 1;
  return ctc;
}

void ContractorIbexPolytopeMt::Prune(ContractorStatus* cs) const {
  ContractorIbexPolytope* const ctc{GetCtcOrCreate(cs->box())};
  DREAL_ASSERT(ctc && !is_dummy());
  return ctc->Prune(cs);
}

ostream& ContractorIbexPolytopeMt::display(ostream& os) const {
  os << "IbexPolytopeMt(";
  for (const Formula& f : formulas_) {
    os << f << ";";
  }
  return os << ")";
}

bool ContractorIbexPolytopeMt::is_dummy() const {
  DREAL_ASSERT(ctc_ready_[0]);
  return ctcs_[0]->is_dummy();
}

}  // namespace dreal
