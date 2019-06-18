#include "dreal/contractor/contractor_id.h"

using std::ostream;

namespace dreal {
ContractorId::ContractorId(const Config& config)
    : ContractorCell{Contractor::Kind::ID, config},
      input_{ibex::BitSet::empty(1) /* this is meaningless */} {}

const ibex::BitSet& ContractorId::input() const { return input_; }

ibex::BitSet& ContractorId::mutable_input() { return input_; }

void ContractorId::Prune(ContractorStatus*) const {
  // No op.
}
ostream& ContractorId::display(ostream& os) const { return os << "ID()"; }

}  // namespace dreal
