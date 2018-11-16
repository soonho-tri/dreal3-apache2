#include "dreal/contractor/contractor_sampling.h"

#include <utility>

#include "dreal/util/assert.h"
#include "dreal/util/logging.h"

using std::move;
using std::ostream;
using std::uniform_real_distribution;
using std::vector;

namespace dreal {

ContractorSampling::ContractorSampling(std::vector<Formula> formulas,
                                       const Box& box, const Config& config)
    : ContractorCell{Contractor::Kind::SAMPLING,
                     ibex::BitSet::empty(box.size()), config},
      formulas_{move(formulas)},
      rg_{config.random_seed()} {
  ibex::BitSet& input{mutable_input()};
  for (const Formula& f : formulas_) {
    for (const Variable& v : f.GetFreeVariables()) {
      input.add(box.index(v));
    }
  }
}

void ContractorSampling::SamplePoint(const Box& box,
                                     Environment* const env) const {
  const ibex::IntervalVector& values{box.interval_vector()};
  for (int i{0}; i < values.size(); i++) {
    const ibex::Interval& iv{values[i]};
    double const lb{iv.lb()};
    double const ub{iv.ub()};
    if (lb != ub) {
      uniform_real_distribution<double> m_dist(lb, ub);
      (*env)[box.variable(i)] = m_dist(rg_);
    } else {
      (*env)[box.variable(i)] = lb;
    }
  }
}

void ContractorSampling::Prune(ContractorStatus* cs) const {
  // Sample a point from the box.
  Box& box = cs->mutable_box();
  Environment env;
  SamplePoint(box, &env);

  // Evaluates formulas at the point
  bool evaulation_result{true};
  for (const Formula& f : formulas_) {
    if (!f.Evaluate(env)) {
      evaulation_result = false;
      break;
    }
  }

  if (evaulation_result) {
    // shrink the box to the point.
    for (const auto& p : env) {
      const Variable& var{p.first};
      const double val{p.second};
      box[var] = val;
    }
    // update the output.
    cs->mutable_output().fill(0, box.size() - 1);
  } else {
    // do nothing.
  }
}

ostream& ContractorSampling::display(ostream& os) const {
  os << "Sampling(";
  for (const Formula& f : formulas_) {
    os << f << ", ";
  }
  return os << ")";
}

}  // namespace dreal
