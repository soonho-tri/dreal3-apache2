#include "dreal/contractor/contractor_seq.h"

#include <utility>

#include "dreal/util/assert.h"

using std::ostream;
using std::vector;

namespace dreal {

ContractorSeq::ContractorSeq(vector<Contractor> contractors,
                             const Config& config)
    : ContractorCell{Contractor::Kind::SEQ, config},
      input_{ibex::BitSet::empty(ComputeInputSize(contractors))},
      contractors_{std::move(contractors)} {
  DREAL_ASSERT(!contractors_.empty());
  for (const Contractor& c : contractors_) {
    input_ |= c.input();
    if (c.include_forall()) {
      set_include_forall();
    }
  }
}

const ibex::BitSet& ContractorSeq::input() const { return input_; }

ibex::BitSet& ContractorSeq::mutable_input() { return input_; }

void ContractorSeq::Prune(ContractorStatus* cs) const {
  for (const Contractor& c : contractors_) {
    c.Prune(cs);
    if (cs->box().empty()) {
      return;
    }
  }
}

ostream& ContractorSeq::display(ostream& os) const {
  os << "Seq(";
  for (const Contractor& c : contractors_) {
    os << c << ", ";
  }
  return os << ")";
}

const std::vector<Contractor>& ContractorSeq::contractors() const {
  return contractors_;
}

}  // namespace dreal
