#include "dreal/util/expression_decomposer.h"

#include <string>
#include <utility>

namespace dreal {

using std::function;
using std::pair;
using std::set;
using std::to_string;

Formula ExpressionDecomposer::Decompose(const Formula& f) {
  Formula new_f{Visit(f)};
  set<Formula> new_set{new_equalities_.begin(), new_equalities_.end()};
  new_f = new_f && make_conjunction(new_set);
  return new_f;
}

void ExpressionDecomposer::Push() {
  cache_.push();
  new_variables_.push();
  new_equalities_.push();
}

void ExpressionDecomposer::Pop() {
  new_equalities_.pop();
  new_variables_.pop();
  cache_.pop();
}

Expression ExpressionDecomposer::GetVariable(const Expression& e) {
  auto it = cache_.find(e);
  if (it != cache_.end()) {
    return it->second;
  }
  it = cache_.find(-e);
  if (it != cache_.end()) {
    return -it->second;
  }
  const Variable v{"var" + to_string(count_++)};
  cache_.insert(e, v);
  new_variables_.push_back(v);
  new_equalities_.push_back(v == e);
  return v;
}

Expression ExpressionDecomposer::Visit(const Expression& e) {
  return VisitExpression<Expression>(this, e);
}

Expression ExpressionDecomposer::VisitVariable(const Expression& e) {
  return e;
}

Expression ExpressionDecomposer::VisitConstant(const Expression& e) {
  return e;
}

Expression ExpressionDecomposer::VisitRealConstant(const Expression& e) {
  return e;
}

Expression ExpressionDecomposer::VisitAddition(const Expression& e) {
  Expression new_e{get_constant_in_addition(e)};
  for (const pair<const Expression, double>& p :
       get_expr_to_coeff_map_in_addition(e)) {
    new_e += Visit(p.first) * p.second;
  }
  return GetVariable(new_e);
}

Expression ExpressionDecomposer::VisitMultiplication(const Expression& e) {
  Expression new_e{get_constant_in_multiplication(e)};
  for (const pair<const Expression, Expression>& p :
       get_base_to_exponent_map_in_multiplication(e)) {
    new_e *= Visit(pow(Visit(p.first), Visit(p.second)));
  }
  return GetVariable(new_e);
}

Expression ExpressionDecomposer::VisitUnary(
    const Expression& e, const function<Expression(const Expression&)>& f) {
  const Expression& arg{get_argument(e)};
  return GetVariable(f(Visit(arg)));
}

Expression ExpressionDecomposer::VisitLog(const Expression& e) {
  return VisitUnary(e, log);
}

Expression ExpressionDecomposer::VisitAbs(const Expression& e) {
  return VisitUnary(e, abs);
}

Expression ExpressionDecomposer::VisitExp(const Expression& e) {
  return VisitUnary(e, exp);
}

Expression ExpressionDecomposer::VisitSqrt(const Expression& e) {
  return VisitUnary(e, sqrt);
}

Expression ExpressionDecomposer::VisitSin(const Expression& e) {
  return VisitUnary(e, sin);
}

Expression ExpressionDecomposer::VisitCos(const Expression& e) {
  return VisitUnary(e, cos);
}

Expression ExpressionDecomposer::VisitTan(const Expression& e) {
  return VisitUnary(e, tan);
}

Expression ExpressionDecomposer::VisitAsin(const Expression& e) {
  return VisitUnary(e, asin);
}

Expression ExpressionDecomposer::VisitAcos(const Expression& e) {
  return VisitUnary(e, acos);
}

Expression ExpressionDecomposer::VisitAtan(const Expression& e) {
  return VisitUnary(e, atan);
}

Expression ExpressionDecomposer::VisitSinh(const Expression& e) {
  return VisitUnary(e, sinh);
}

Expression ExpressionDecomposer::VisitCosh(const Expression& e) {
  return VisitUnary(e, cosh);
}

Expression ExpressionDecomposer::VisitTanh(const Expression& e) {
  return VisitUnary(e, tanh);
}

Expression ExpressionDecomposer::VisitBinary(
    const Expression& e,
    const function<Expression(const Expression&, const Expression&)>& f) {
  const Expression& arg1{get_first_argument(e)};
  const Expression& arg2{get_second_argument(e)};
  return GetVariable(f(Visit(arg1), Visit(arg2)));
}

Expression ExpressionDecomposer::VisitDivision(const Expression& e) {
  return VisitBinary(
      e, [](const Expression& e1, const Expression& e2) { return e1 / e2; });
}

Expression ExpressionDecomposer::VisitPow(const Expression& e) {
  return VisitBinary(e, pow);
}

Expression ExpressionDecomposer::VisitAtan2(const Expression& e) {
  return VisitBinary(e, atan2);
}

Expression ExpressionDecomposer::VisitMin(const Expression& e) {
  return VisitBinary(e, min);
}

Expression ExpressionDecomposer::VisitMax(const Expression& e) {
  return VisitBinary(e, max);
}

Expression ExpressionDecomposer::VisitIfThenElse(const Expression& e) {
  const Formula& cond{get_conditional_formula(e)};
  const Expression& e1{get_then_expression(e)};
  const Expression& e2{get_else_expression(e)};
  return GetVariable(if_then_else(Visit(cond), Visit(e1), Visit(e2)));
}

Expression ExpressionDecomposer::VisitUninterpretedFunction(
    const Expression& e) {
  return e;
}

Formula ExpressionDecomposer::Visit(const Formula& f) {
  return VisitFormula<Formula>(this, f);
}

Formula ExpressionDecomposer::VisitFalse(const Formula& f) { return f; }

Formula ExpressionDecomposer::VisitTrue(const Formula& f) { return f; }

Formula ExpressionDecomposer::VisitVariable(const Formula& f) { return f; }

Formula ExpressionDecomposer::VisitEqualTo(const Formula& f) {
  const Expression& e1(get_lhs_expression(f));
  const Expression& e2(get_rhs_expression(f));
  return Visit(e1) == Visit(e2);
}

Formula ExpressionDecomposer::VisitNotEqualTo(const Formula& f) {
  const Expression& e1(get_lhs_expression(f));
  const Expression& e2(get_rhs_expression(f));
  return Visit(e1) == Visit(e2);
}

Formula ExpressionDecomposer::VisitGreaterThan(const Formula& f) {
  const Expression& e1(get_lhs_expression(f));
  const Expression& e2(get_rhs_expression(f));
  return Visit(e1) > Visit(e2);
}

Formula ExpressionDecomposer::VisitGreaterThanOrEqualTo(const Formula& f) {
  const Expression& e1(get_lhs_expression(f));
  const Expression& e2(get_rhs_expression(f));
  return Visit(e1) >= Visit(e2);
}

Formula ExpressionDecomposer::VisitLessThan(const Formula& f) {
  const Expression& e1(get_lhs_expression(f));
  const Expression& e2(get_rhs_expression(f));
  return Visit(e1) < Visit(e2);
}

Formula ExpressionDecomposer::VisitLessThanOrEqualTo(const Formula& f) {
  const Expression& e1(get_lhs_expression(f));
  const Expression& e2(get_rhs_expression(f));
  return Visit(e1) <= Visit(e2);
}

Formula ExpressionDecomposer::VisitConjunction(const Formula& f) {
  set<Formula> new_set;
  for (const Formula& conjunct : get_operands(f)) {
    new_set.insert(Visit(conjunct));
  }
  return make_conjunction(new_set);
}

Formula ExpressionDecomposer::VisitDisjunction(const Formula& f) {
  set<Formula> new_set;
  for (const Formula& disjunct : get_operands(f)) {
    new_set.insert(Visit(disjunct));
  }
  return make_disjunction(new_set);
}

Formula ExpressionDecomposer::VisitNegation(const Formula& f) {
  return !Visit(get_operand(f));
}

Formula ExpressionDecomposer::VisitForall(const Formula& f) {
  // throw std::runtime_error("implement");
  return f;
}

}  // namespace dreal
