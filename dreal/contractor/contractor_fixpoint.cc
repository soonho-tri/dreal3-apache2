#include "dreal/contractor/contractor_fixpoint.h"

#include <utility>

#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/interrupt.h"
#include "dreal/util/logging.h"

using std::ostream;
using std::vector;

namespace dreal {

ContractorFixpoint::ContractorFixpoint(TerminationCondition term_cond,
                                       vector<Contractor> contractors,
                                       const Config& config)
    : ContractorCell{Contractor::Kind::FIXPOINT, config},
      input_(ibex::BitSet::empty(ComputeInputSize(contractors))),
      term_cond_{std::move(term_cond)},
      contractors_{std::move(contractors)} {
  DREAL_ASSERT(!contractors_.empty());
  for (const Contractor& c : contractors_) {
    input_ |= c.input();
    if (c.include_forall()) {
      set_include_forall();
    }
  }
}

const ibex::BitSet& ContractorFixpoint::input() const { return input_; }

ibex::BitSet& ContractorFixpoint::mutable_input() { return input_; }

void ContractorFixpoint::Prune(ContractorStatus* cs) const {
  const Box::IntervalVector& iv{cs->box().interval_vector()};
  Box::IntervalVector old_iv{iv};
  do {
    // Note that 'DREAL_CHECK_INTERRUPT' is only defined in setup.py,
    // when we build dReal python package.
#ifdef DREAL_CHECK_INTERRUPT
    if (g_interrupted) {
      DREAL_LOG_DEBUG("KeyboardInterrupt(SIGINT) Detected.");
      throw std::runtime_error("KeyboardInterrupt(SIGINT) Detected.");
    }
#endif
    old_iv = iv;
    for (const Contractor& ctc : contractors_) {
      ctc.Prune(cs);
      if (iv.is_empty()) {
        return;
      }
    }
  } while (!term_cond_(old_iv, iv));
}

ostream& ContractorFixpoint::display(ostream& os) const {
  os << "Fixpoint(";
  for (const Contractor& c : contractors_) {
    os << c << ", ";
  }
  return os << ")";
}

}  // namespace dreal
