#pragma once

#include <functional>
#include <set>
#include <unordered_map>

#include "dreal/symbolic/symbolic.h"
#include "dreal/util/scoped_unordered_map.h"
#include "dreal/util/scoped_vector.h"

namespace dreal {

class ExpressionDecomposer {
 public:
  /// Decomposes a composite expression such as `z = sin(x) + cos(y)`
  /// into a conjunction of simple expressions, `z = v1 + v2`, `v1 =
  /// sin(x)`, `v2 = cos(y)`.
  Formula Decompose(const Formula& f);

  /// Returns a const reference to the set of newly introduced variables during
  /// the decomposition.
  const ScopedVector<Variable>& NewVariables() const { return new_variables_; }

  /// Stores the current state.
  void Push();

  /// Restores the previous state.
  void Pop();

 private:
  // Create a new variable `v` for `e` and return `v`.  It also add a
  // new equality `v == e`.  This method uses `cache_` to share the
  // same variable `v` for the expression `e`.
  Expression GetVariable(const Expression& e);

  // Handle expressions.
  Expression Visit(const Expression& e);
  Expression VisitVariable(const Expression& e);
  Expression VisitConstant(const Expression& e);
  Expression VisitRealConstant(const Expression& e);
  Expression VisitAddition(const Expression& e);
  Expression VisitMultiplication(const Expression& e);

  Expression VisitUnary(const Expression& e,
                        const std::function<Expression(const Expression&)>& f);
  Expression VisitLog(const Expression& e);
  Expression VisitAbs(const Expression& e);
  Expression VisitExp(const Expression& e);
  Expression VisitSqrt(const Expression& e);
  Expression VisitSin(const Expression& e);
  Expression VisitCos(const Expression& e);
  Expression VisitTan(const Expression& e);
  Expression VisitAsin(const Expression& e);
  Expression VisitAcos(const Expression& e);
  Expression VisitAtan(const Expression& e);
  Expression VisitSinh(const Expression& e);
  Expression VisitCosh(const Expression& e);
  Expression VisitTanh(const Expression& e);

  Expression VisitBinary(
      const Expression& e,
      const std::function<Expression(const Expression&, const Expression&)>& f);
  Expression VisitDivision(const Expression& e);
  Expression VisitPow(const Expression& e);
  Expression VisitAtan2(const Expression& e);
  Expression VisitMin(const Expression& e);
  Expression VisitMax(const Expression& e);

  Expression VisitIfThenElse(const Expression& e);
  Expression VisitUninterpretedFunction(const Expression& e);

  // Handle formula
  Formula Visit(const Formula& f);
  Formula VisitFalse(const Formula& f);
  Formula VisitTrue(const Formula& f);
  Formula VisitVariable(const Formula& f);
  Formula VisitEqualTo(const Formula& f);
  Formula VisitNotEqualTo(const Formula& f);
  Formula VisitGreaterThan(const Formula& f);
  Formula VisitGreaterThanOrEqualTo(const Formula& f);
  Formula VisitLessThan(const Formula& f);
  Formula VisitLessThanOrEqualTo(const Formula& f);
  Formula VisitConjunction(const Formula& f);
  Formula VisitDisjunction(const Formula& f);
  Formula VisitNegation(const Formula& f);
  Formula VisitForall(const Formula& f);

  // Makes VisitFormula a friend of this class so that it can use private
  // operator()s.
  friend Formula drake::symbolic::VisitFormula<Formula>(ExpressionDecomposer*,
                                                        const Formula&);
  // Makes VisitExpression a friend of this class so that it can use private
  // operator()s.
  friend Expression drake::symbolic::VisitExpression<Expression>(
      ExpressionDecomposer*, const Expression&);

  // Members
  int count_{0};
  ScopedVector<Formula> new_equalities_;
  ScopedVector<Variable> new_variables_;
  ScopedUnorderedMap<Expression, Variable> cache_;
};

}  // namespace dreal
