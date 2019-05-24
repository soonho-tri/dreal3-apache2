#include "dreal/contractor/contractor_ibex_polytope_mt.h"

#include <utility>

#include "dreal/util/logging.h"

using std::make_unique;
using std::ostream;
using std::vector;

namespace dreal {
namespace {
std::size_t get_thread_id() noexcept {
  static std::atomic<std::size_t> id{0};
  thread_local const std::size_t tid{++id};
  return tid;
}

}  // namespace
ContractorIbexPolytopeMt::ContractorIbexPolytopeMt(vector<Formula> formulas,
                                                   const Box& box,
                                                   const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_POLYTOPE,
                     ibex::BitSet::empty(box.size()), config},
      formulas_{std::move(formulas)},
      config_{config},
      ctcs_ready_(config.number_of_jobs(), 0),
      ctcs_(config.number_of_jobs()) {
  DREAL_LOG_DEBUG("ContractorIbexPolytopeMt::ContractorIbexPolytopeMt");
  GetCtcOrCreate(box);
}

ContractorIbexPolytope* ContractorIbexPolytopeMt::GetCtcOrCreate(
    const Box& box) const {
  thread_local const std::size_t id{get_thread_id()};

  if (ctcs_ready_[id]) {
    return ctcs_[id].get();
  }
  ctcs_[id].reset(new ContractorIbexPolytope(formulas_, box, config_));
  ctcs_ready_[id] = 1;
  return ctcs_[id].get();
}

void ContractorIbexPolytopeMt::Prune(ContractorStatus* cs) const {
  ContractorIbexPolytope* const ctc{GetCtcOrCreate(cs->box())};
  if (ctc) {
    return ctc->Prune(cs);
  }
}

ostream& ContractorIbexPolytopeMt::display(ostream& os) const {
  os << "IbexPolytopeMt(";
  for (const Formula& f : formulas_) {
    os << f << ";";
  }
  return os << ")";
}

}  // namespace dreal
