#include "dreal/contractor/contractor_ibex_polytope_mt.h"

#include <utility>

#include "dreal/util/logging.h"

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
      config_{config},
      ctc_map_{static_cast<
          cuckoohash_map<std::thread::id, ContractorIbexPolytope*>::size_type>(
          config_.number_of_jobs())} {
  DREAL_LOG_DEBUG("ContractorIbexPolytopeMt::ContractorIbexPolytopeMt");
  ctcs_.reserve(config.number_of_jobs());
  GetCtc(box);
}

ContractorIbexPolytopeMt::~ContractorIbexPolytopeMt() {
  for (const ContractorIbexPolytope* ctc : ctcs_) {
    delete ctc;
  }
}

ContractorIbexPolytope* ContractorIbexPolytopeMt::GetCtc(const Box& box) const {
  thread_local const std::thread::id id{std::this_thread::get_id()};
  if (ctc_map_.contains(id)) {
    return ctc_map_.find(id);
  }
  ContractorIbexPolytope* const ctc{
      new ContractorIbexPolytope{formulas_, box, config_}};
  ctc_map_.insert(id, ctc);  // <---- 1
  {
    std::lock_guard<std::mutex> guard{m_};
    ctcs_.push_back(ctc);
    return ctc;
  }
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
