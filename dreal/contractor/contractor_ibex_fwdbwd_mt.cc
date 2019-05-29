#include "dreal/contractor/contractor_ibex_fwdbwd_mt.h"

#include <utility>

#include "dreal/util/logging.h"
#include "dreal/util/timer.h"

using std::make_unique;
using std::ostream;
using std::thread;
using std::unique_ptr;

namespace dreal {

ContractorIbexFwdbwdMt::ContractorIbexFwdbwdMt(Formula f, const Box& box,
                                               const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_FWDBWD,
                     ibex::BitSet::empty(box.size()), config},
      f_{std::move(f)},
      config_{config},
      ctc_map_(config.number_of_jobs() * LIBCUCKOO_DEFAULT_SLOT_PER_BUCKET) {
  DREAL_LOG_DEBUG("ContractorIbexFwdbwdMt::ContractorIbexFwdbwdMt");
  ContractorIbexFwdbwd* const ctc{GetCtcOrCreate(box)};
  // Build input.
  if (ctc) {
    mutable_input() = ctc->input();
  }
}

ContractorIbexFwdbwd* ContractorIbexFwdbwdMt::GetCtcOrCreate(
    const Box& box) const {
  thread_local const std::thread::id id{std::this_thread::get_id()};
  ContractorIbexFwdbwd* ctc{nullptr};
  if (ctc_map_.find_fn(id, [&ctc](const unique_ptr<ContractorIbexFwdbwd>& v) {
        ctc = v.get();
      })) {
    return ctc;
  }
  Timer tt;
  tt.start();
  auto ctc_unique_ptr = make_unique<ContractorIbexFwdbwd>(f_, box, config_);
  ctc = ctc_unique_ptr.get();
  ctc_map_.insert(id, std::move(ctc_unique_ptr));
  tt.pause();
  DREAL_LOG_CRITICAL("FWDBWD {}", tt.seconds());
  return ctc;
}

void ContractorIbexFwdbwdMt::Prune(ContractorStatus* cs) const {
  ContractorIbexFwdbwd* const ctc{GetCtcOrCreate(cs->box())};
  if (ctc) {
    return ctc->Prune(cs);
  }
}

ostream& ContractorIbexFwdbwdMt::display(ostream& os) const {
  return os << "IbexFwdbwd(" << f_ << ")";
}
}  // namespace dreal
