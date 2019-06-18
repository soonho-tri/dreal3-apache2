#include "dreal/contractor/contractor_join.h"

#include <utility>

#include "dreal/util/assert.h"

using std::ostream;
using std::vector;

namespace dreal {

ContractorJoin::ContractorJoin(vector<Contractor> contractors,
                               const Config& config)
    : ContractorCell{Contractor::Kind::JOIN, config},
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

const ibex::BitSet& ContractorJoin::input() const { return input_; }

ibex::BitSet& ContractorJoin::mutable_input() { return input_; }

void ContractorJoin::Prune(ContractorStatus* cs) const {
  ContractorStatus saved_original{*cs};
  cs->mutable_box().set_empty();
  for (const Contractor& contractor : contractors_) {
    ContractorStatus state_i{saved_original};
    contractor.Prune(&state_i);
    cs->InplaceJoin(state_i);
  }
}

ostream& ContractorJoin::display(ostream& os) const {
  os << "Join(";
  for (const Contractor& c : contractors_) {
    os << c << ", ";
  }
  return os << ")";
}

}  // namespace dreal
