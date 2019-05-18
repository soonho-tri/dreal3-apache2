#include "dreal/contractor/contractor_ibex_polytope_mt.h"

#include <utility>

#include "dreal/util/logging.h"

using std::make_unique;
using std::ostream;
using std::vector;

namespace dreal {

//---------------------------------------
// Implementation of ContractorIbexPolytope
//---------------------------------------
ContractorIbexPolytopeMt::ContractorIbexPolytopeMt(vector<Formula> formulas,
                                                   const Box& box,
                                                   const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_POLYTOPE,
                     ibex::BitSet::empty(box.size()), config},
      formulas_{std::move(formulas)},
      config_{config} {
  DREAL_LOG_DEBUG("ContractorIbexPolytopeMt::ContractorIbexPolytopeMt");
  GetCtc(box);
}

ContractorIbexPolytope* ContractorIbexPolytopeMt::GetCtc(const Box& box) const {
  thread_local const std::thread::id id{std::this_thread::get_id()};
  ContractorIbexPolytope* ctc{nullptr};
  if (ctc_map_.find_fn(
          id, [&ctc](const std::unique_ptr<ContractorIbexPolytope>& v) {
            ctc = v.get();
          })) {
    return ctc;
  }
  auto ctc_unique_ptr =
      make_unique<ContractorIbexPolytope>(formulas_, box, config_);
  ctc = ctc_unique_ptr.get();
  ctc_map_.insert(id, std::move(ctc_unique_ptr));
  return ctc;
}

void ContractorIbexPolytopeMt::Prune(ContractorStatus* cs) const {
  ContractorIbexPolytope* const ctc{GetCtc(cs->box())};
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
