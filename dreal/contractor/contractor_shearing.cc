#include "dreal/contractor/contractor_shearing.h"

#include <algorithm>
#include <utility>

#include "dreal/util/assert.h"
#include "dreal/util/logging.h"
#include "dreal/util/math.h"
#include "dreal/util/stat.h"

#include "dreal/util/expression_evaluator.h"

namespace dreal {

ContractorShearing::ContractorShearing(Formula f, const Box& box,
                                       const Config& config,
                                       const Contractor::ShearingMethod method,
                                       const int n, const double alpha)
    : ContractorCell{Contractor::Kind::SHEARING,
                     ibex::BitSet::empty(box.size()), config},
      f_{std::move(f)},
      method_{method},
      n_{n},
      alpha_{alpha} {
  DREAL_ASSERT(is_relational(f_));
  DREAL_ASSERT(!is_not_equal_to(f_));

  e_ = get_lhs_expression(f_) - get_rhs_expression(f_);

  if (is_equal_to(f_)) {
    rop_ = RelationalOperator::EQ;
  } else if (is_greater_than(f_)) {
    rop_ = RelationalOperator::GT;
  } else if (is_greater_than_or_equal_to(f_)) {
    rop_ = RelationalOperator::GEQ;
  } else if (is_less_than(f_)) {
    rop_ = RelationalOperator::LT;
  } else if (is_less_than_or_equal_to(f_)) {
    rop_ = RelationalOperator::LEQ;
  }

  // Build input.
  ibex::BitSet& input{mutable_input()};
  for (const Variable& var : f_.GetFreeVariables()) {
    input.add(box.index(var));
  }
}

namespace {
double CheckIfBoxCanBeRemoved(const Expression& e, const RelationalOperator rop,
                              const Box& b,
                              const Contractor::ShearingMethod method) {
  Box::Interval result;
  switch (method) {
    case Contractor::ShearingMethod::Natural:
      result = Eval(e, b);
      break;
    case Contractor::ShearingMethod::Taylor1:
      result = Taylor1Eval(e, b);
      break;
    case Contractor::ShearingMethod::Taylor2:
      result = Taylor2Eval(e, b);
      break;
  }

  switch (rop) {
    case RelationalOperator::EQ:
      if (result.lb() > 0.0) {
        return result.lb();
      }
      if (result.ub() < 0.0) {
        return -result.ub();
      }
      break;
    case RelationalOperator::NEQ:
      return -1.0;
    case RelationalOperator::GEQ:
      if (result.ub() < 0.0) {
        return -result.ub();
      }
      break;
    case RelationalOperator::GT:
      if (result.ub() <= 0.0) {
        return -result.ub();
      }
      break;
    case RelationalOperator::LEQ:
      if (result.lb() > 0.0) {
        return result.lb();
      }
      break;
    case RelationalOperator::LT:
      if (result.lb() >= 0.0) {
        return result.lb();
      }
      break;
  }
  // Should not reach here.
  return -1;
}
}  // namespace

void ContractorShearing::Prune(ContractorStatus* cs) const {
  // Pick a dimension: For now, it always picks up the dimension with
  // the maximum diameter.
  const std::pair<double, int> max_diam_info{
      cs->box().MaxDiam(e_.GetVariables())};
  const int the_dim{max_diam_info.second};
  if (the_dim < 0) {
    return;
  }
  const double max_diam{cs->box()[the_dim].diam()};
  const double init_step_size{max_diam / n_};

  Box b{cs->box()};
  double lb{cs->box()[the_dim].lb()};
  const double ub{cs->box()[the_dim].ub()};

  bool changed{false};
  // Left-to-right sweep
  {
    double step_size = init_step_size;
    for (double lb_i = lb; lb_i <= ub; lb_i += step_size) {
      const double ub_i = std::min(lb_i + step_size, ub);
      b[the_dim] = Box::Interval{lb_i, ub_i};
      const double robust_value{CheckIfBoxCanBeRemoved(e_, rop_, b, method_)};
      if (robust_value >= 0.0) {
        cs->mutable_box()[the_dim] = Box::Interval{ub_i, ub};
        changed = true;
        step_size = step_size + (robust_value - step_size) * alpha_;
      } else {
        break;
      }
    }
  }
  // Right-to-left sweep
  {
    double step_size = init_step_size;
    lb = cs->box()[the_dim].lb();
    for (double ub_i = ub; ub_i >= lb; ub_i -= step_size) {
      const double lb_i = std::max(lb, ub_i - step_size);
      b[the_dim] = Box::Interval{lb_i, ub_i};
      const double robust_value{CheckIfBoxCanBeRemoved(e_, rop_, b, method_)};
      if (robust_value >= 0.0) {
        cs->mutable_box()[the_dim] = Box::Interval{lb, lb_i};
        changed = true;
        step_size = step_size + (robust_value - step_size) * alpha_;
      } else {
        break;
      }
    }
  }
  if (changed) {
    cs->AddUsedConstraint(f_);
    if (cs->box().empty()) {
      cs->mutable_output().fill(0, cs->box().size() - 1);
    } else {
      cs->mutable_output().add(the_dim);
    }
  }
}

std::ostream& ContractorShearing::display(std::ostream& os) const {
  return os << "ContractorShearing(" << e_ << " " << rop_ << " 0)";
}

}  // namespace dreal
