#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dreal/symbolic/symbolic.h"
#include "dreal/util/assert.h"
#include "dreal/util/box.h"
#include "dreal/util/eigen.h"
#include "dreal/util/exception.h"
#include "dreal/util/math.h"

namespace dreal {

class EvaluationVisitor {
 public:
  explicit EvaluationVisitor(const std::vector<Variable>& variables) {
    int i = 0;
    for (const Variable& var : variables) {
      variable_id_to_idx_[var.get_id()] = i++;
    }
  }

  /// Evaluates the expression with @p box.
  template <typename Scalar>
  Scalar Evaluate(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    return ::dreal::drake::symbolic::VisitExpression<Scalar>(this, e, x);
  }

  /// Evaluates the expression with @p box.
  template <typename Scalar>
  VectorX<Scalar> Evaluate(
      const Eigen::Ref<const VectorX<Expression>>& expressions,
      const Eigen::Ref<const VectorX<Scalar>>& x) const {
    return expressions.unaryExpr(
        [&x, this](const Expression& e) { return Evaluate(e, x); });
  }

  template <typename Scalar>
  Scalar VisitVariable(const Expression& e,
                       const Eigen::Ref<const VectorX<Scalar>>& x) const {
    const Variable& var{get_variable(e)};
    return x[variable_id_to_idx_.at(var.get_id())];
  }

  template <typename Scalar>
  Scalar VisitConstant(const Expression& e,
                       const Eigen::Ref<const VectorX<Scalar>>&) const {
    return Scalar{get_constant_value(e)};
  }

  template <typename Scalar>
  Scalar VisitRealConstant(const Expression& e,
                           const Eigen::Ref<const VectorX<Scalar>>&) const {
    return Scalar{get_constant_value(e)};
  }

  /// Specialization for Box::Interval
  Box::Interval VisitRealConstant(
      const Expression& e,
      const Eigen::Ref<const VectorX<Box::Interval>>&) const {
    const double lb{get_lb_of_real_constant(e)};
    const double ub{get_ub_of_real_constant(e)};
    return Box::Interval{lb, ub};
  }

  template <typename Scalar>
  Scalar VisitAddition(const Expression& e,
                       const Eigen::Ref<const VectorX<Scalar>>& x) const {
    Scalar ret{get_constant_in_addition(e)};
    for (const std::pair<const Expression, double>& p :
         get_expr_to_coeff_map_in_addition(e)) {
      ret += Evaluate(p.first, x) * p.second;
    }
    return ret;
  }

  template <typename Scalar>
  Scalar VisitMultiplication(const Expression& e,
                             const Eigen::Ref<const VectorX<Scalar>>& x) const {
    Scalar ret{get_constant_in_multiplication(e)};
    for (const std::pair<const Expression, Expression>& p :
         get_base_to_exponent_map_in_multiplication(e)) {
      ret *= VisitPow(p.first, p.second, x);
    }
    return ret;
  }

  template <typename Scalar>
  Scalar VisitDivision(const Expression& e,
                       const Eigen::Ref<const VectorX<Scalar>>& x) const {
    return Evaluate(get_first_argument(e), x) /
           Evaluate(get_second_argument(e), x);
  }

  template <typename Scalar>
  Scalar VisitLog(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::log;
    return log(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitAbs(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::abs;
    return abs(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitExp(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::exp;
    return exp(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitSqrt(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::sqrt;
    return sqrt(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitPow(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    return VisitPow(get_first_argument(e), get_second_argument(e), x);
  }

  template <typename Scalar>
  Scalar VisitPow(const Expression& e1, const Expression& e2,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::pow;
    return pow(Evaluate(e1, x), Evaluate(e2, x));
  }

  /// Specialization for Box::Interval
  Box::Interval VisitPow(
      const Expression& e1, const Expression& e2,
      const Eigen::Ref<const VectorX<Box::Interval>>& x) const {
    const Box::Interval i1{Evaluate(e1, x)};
    const Box::Interval i2{Evaluate(e2, x)};
    if (i2.is_degenerated() && !i2.is_empty()) {
      // This indicates that this interval is a point.
      DREAL_ASSERT(i2.lb() == i2.ub());
      const double point{i2.lb()};
      if (is_integer(point)) {
        if (point == 2.0) {
          return sqr(i1);
        } else {
          return pow(i1, static_cast<int>(point));
        }
      } else {
        return pow(i1, point);
      }
    } else {
      return pow(i1, i2);
    }
  }

  template <typename Scalar>
  Scalar VisitSin(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::sin;
    return sin(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitCos(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::cos;
    return cos(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitTan(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::tan;
    return tan(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitAsin(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::asin;
    return asin(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitAcos(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::acos;
    return acos(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitAtan(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::atan;
    return atan(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitAtan2(const Expression& e,
                    const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::atan2;
    return atan2(Evaluate(get_first_argument(e), x),
                 Evaluate(get_second_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitSinh(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::sinh;
    return sinh(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitCosh(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::cosh;
    return cosh(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitTanh(const Expression& e,
                   const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::tanh;
    return tanh(Evaluate(get_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitMin(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::min;
    return min(Evaluate(get_first_argument(e), x),
               Evaluate(get_second_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitMax(const Expression& e,
                  const Eigen::Ref<const VectorX<Scalar>>& x) const {
    using std::max;
    return max(Evaluate(get_first_argument(e), x),
               Evaluate(get_second_argument(e), x));
  }

  template <typename Scalar>
  Scalar VisitIfThenElse(const Expression&,
                         const Eigen::Ref<const VectorX<Scalar>>&) const {
    throw DREAL_RUNTIME_ERROR("If-then-else expression is not supported yet.");
  }

  template <typename Scalar>
  Scalar VisitUninterpretedFunction(
      const Expression&, const Eigen::Ref<const VectorX<Scalar>>&) const {
    throw DREAL_RUNTIME_ERROR("Uninterpreted function is not supported.");
  }

 private:
  std::unordered_map<Variable::Id, int> variable_id_to_idx_;
};  // namespace dreal

// Evaluates the expression @p e at @p x. @p variables is provided to
// specify the order of variables. For example, if variables = {v0, v1, v2}
// and x = [1, 2, 3], then it is interpreted as (v0, v1, v2) = (1, 2, 3).
template <typename Scalar>
Scalar Evaluate(const Expression& e, const std::vector<Variable>& variables,
                const Eigen::Ref<const VectorX<Scalar>>& x) {
  return EvaluationVisitor{variables}.Evaluate(e, x);
}

}  // namespace dreal
