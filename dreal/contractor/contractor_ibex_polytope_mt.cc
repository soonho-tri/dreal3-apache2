#include "dreal/contractor/contractor_ibex_polytope_mt.h"

#include <utility>

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
      config_{config} {
  // ctc_map_(config.number_of_jobs() * LIBCUCKOO_DEFAULT_SLOT_PER_BUCKET) {
  DREAL_LOG_DEBUG("ContractorIbexPolytopeMt::ContractorIbexPolytopeMt");
  ContractorIbexPolytope* const ctc{GetCtcOrCreate(box)};
  // Build input.
  if (ctc) {
    mutable_input() = ctc->input();
  }
}

ContractorIbexPolytope* ContractorIbexPolytopeMt::GetCtcOrCreate(
    const Box& box) const {
  thread_local const std::thread::id id{std::this_thread::get_id()};
  ContractorIbexPolytope* ctc{nullptr};
  if (ctc_map_.find_fn(
          id, [&ctc](const std::unique_ptr<ContractorIbexPolytope>& v) {
            ctc = v.get();
          })) {
    return ctc;
  }
  Timer tt;
  tt.start();
  auto ctc_unique_ptr =
      make_unique<ContractorIbexPolytope>(formulas_, box, config_);
  tt.pause();
  DREAL_LOG_CRITICAL("Polytope {}", tt.seconds());

  ctc = ctc_unique_ptr.get();
  ctc_map_.insert(id, std::move(ctc_unique_ptr));
  return ctc;
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
