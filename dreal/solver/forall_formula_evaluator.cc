#include "dreal/solver/forall_formula_evaluator.h"

#include <limits>
#include <set>
#include <utility>

#include "ThreadPool/ThreadPool.h"

#include "dreal/symbolic/symbolic.h"
#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/logging.h"
#include "dreal/util/optional.h"

namespace dreal {

using std::ostream;
using std::set;
using std::vector;

namespace {

// Given f = [(e₁(x, y) ≥ 0) ∨ ... ∨ (eₙ(x, y) ≥ 0)], build an
// evaluator for each (eᵢ(x, y) ≥ 0) and return a vector of
// evaluators.
vector<RelationalFormulaEvaluator> BuildFormulaEvaluators(
    const set<Formula>& disjuncts) {
  vector<RelationalFormulaEvaluator> evaluators;
  evaluators.reserve(disjuncts.size());
  for (const Formula& disjunct : disjuncts) {
    DREAL_LOG_DEBUG("BuildFormulaEvaluators: disjunct = {}", disjunct);
    DREAL_ASSERT(
        is_relational(disjunct) ||
        (is_negation(disjunct) && is_relational(get_operand(disjunct))));
    evaluators.emplace_back(disjunct);
  }
  return evaluators;
}

vector<RelationalFormulaEvaluator> BuildFormulaEvaluators(const Formula& f) {
  DREAL_LOG_DEBUG("BuildFormulaEvaluators");
  const Formula& quantified_formula{get_quantified_formula(f)};
  DREAL_ASSERT(is_clause(quantified_formula));
  if (is_disjunction(quantified_formula)) {
    return BuildFormulaEvaluators(get_operands(quantified_formula));
  } else {
    return BuildFormulaEvaluators(std::set<Formula>{quantified_formula});
  }
}
}  // namespace

Context& ForallFormulaEvaluator::GetContext() const {
  thread_local const int kThreadId{ThreadPool::get_thread_id()};
  DREAL_ASSERT(0 <= kThreadId &&
               kThreadId <= static_cast<int>(contexts_.size()));
  return contexts_[kThreadId];
}

ForallFormulaEvaluator::ForallFormulaEvaluator(Formula f, const double epsilon,
                                               const double delta,
                                               const int number_of_jobs)
    : FormulaEvaluatorCell{std::move(f)},
      evaluators_{BuildFormulaEvaluators(formula())} {
  DREAL_ASSERT(is_forall(formula()));
  DREAL_LOG_DEBUG("ForallFormulaEvaluator({})", formula());

  Config config;
  config.mutable_precision() = delta;
  contexts_.reserve(number_of_jobs);
  for (int i = 0; i < number_of_jobs; ++i) {
    DREAL_ASSERT(config.number_of_jobs() == 1);
    contexts_.emplace_back(config);
    Context& context{contexts_[i]};
    for (const Variable& exist_var : formula().GetFreeVariables()) {
      context.DeclareVariable(exist_var);
    }
    for (const Variable& forall_var : get_quantified_variables(formula())) {
      context.DeclareVariable(forall_var);
    }
    context.Assert(
        DeltaStrengthen(!get_quantified_formula(formula()), epsilon));
  }
}

FormulaEvaluationResult ForallFormulaEvaluator::operator()(
    const Box& box) const {
  Context& context{GetContext()};
  for (const Variable& v : box.variables()) {
    context.SetInterval(v, box[v].lb(), box[v].ub());
  }
  optional<Box> counterexample = context.CheckSat();
  DREAL_LOG_DEBUG("ForallFormulaEvaluator::operator({})", box);
  if (counterexample) {
    DREAL_LOG_DEBUG("ForallFormulaEvaluator::operator()  --  CE found: ",
                    *counterexample);
    for (const Variable& exist_var : box.variables()) {
      (*counterexample)[exist_var] = box[exist_var];
    }
    double max_diam = 0.0;
    for (const RelationalFormulaEvaluator& evaluator : evaluators_) {
      const FormulaEvaluationResult eval_result = evaluator(*counterexample);
      double diam_i{0.0};
      if (eval_result.type() == FormulaEvaluationResult::Type::UNSAT) {
        diam_i = eval_result.evaluation().mag();
      } else {
        diam_i = eval_result.evaluation().diam();
      }
      if (diam_i > max_diam) {
        max_diam = diam_i;
      }
    }
    return FormulaEvaluationResult{FormulaEvaluationResult::Type::UNKNOWN,
                                   Box::Interval(0.0, max_diam)};
  } else {
    DREAL_LOG_DEBUG("ForallFormulaEvaluator::operator()  --  No CE found: ");
    return FormulaEvaluationResult{FormulaEvaluationResult::Type::VALID,
                                   Box::Interval(0.0, 0.0)};
  }
}

ostream& ForallFormulaEvaluator::Display(ostream& os) const {
  return os << "ForallFormulaEvaluator(" << formula() << ")";
}

const Variables& ForallFormulaEvaluator::variables() const {
  return formula().GetFreeVariables();
}

}  // namespace dreal
