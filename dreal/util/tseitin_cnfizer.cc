#include "dreal/util/tseitin_cnfizer.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <set>
#include <string>
#include <experimental/optional>

#include "dreal/util/assert.h"
#include "dreal/util/exception.h"
#include "dreal/util/logging.h"

namespace dreal {

using std::set;
using std::string;
using std::to_string;
using std::unordered_map;
using std::vector;

using std::experimental::optional;

namespace {
// Forward declarations for the helper functions.
void Cnfize(const Variable& b, const Formula& f, vector<Formula>* clauses);
void CnfizeNegation(const Variable& b, const Formula& f,
                    vector<Formula>* clauses);
void CnfizeConjunction(const Variable& b, const Formula& f,
                       vector<Formula>* clauses);
void CnfizeDisjunction(const Variable& b, const Formula& f,
                       vector<Formula>* clauses);

// Updates the formula susbtitution `subst` : Variable → Formula.
//  - If `clause` is a positive literal `x`, add `x ↦ True` to
//    `subst`.
//  - If `clause` is a negative literal, `¬x`, add `x ↦ False`
//    to `subst`. Otherwise, `subst` is not changed.
//
// Returns the variable if `subst` is updated.
optional<Variable> UpdateSubstitutionIfPure(const Formula& clause,
                                            FormulaSubstitution* subst) {
  if (is_variable(clause)) {
    // Positive literal.
    const Variable var{get_variable(clause)};
    const auto result = subst->emplace(var, Formula::True());
    return result.second ? var : optional<Variable>();
  } else if (is_negation(clause) && is_variable(get_operand(clause))) {
    // Negative literal.
    const Variable var{get_variable(get_operand(clause))};
    const auto result = subst->emplace(var, Formula::False());
    return result.second ? var : optional<Variable>();
  }
  return {};
}

// Eliminates pure literals in `clauses`. It also eliminates any true
// clauses in the vector. The `map` is used to check if a variable is
// temporary or not.
void EliminatePureLiterals(const std::map<Variable, Formula>& map,
                           vector<Formula>* clauses) {
  // Repeat:
  //   1. Find a unit clause in clauses. If not exist, quit.
  //   2. Propagate pure literals. Remove clause if it's a temporary variable
  //      or true.
  FormulaSubstitution subst;
  optional<Variable> subst_result;
  do {
    auto it = clauses->begin();
    while (it != clauses->end()) {
      subst_result = UpdateSubstitutionIfPure(*it, &subst);
      if (!subst_result && !is_atomic(*it)) {
        *it = it->Substitute(subst);
        // Need to check if clause becomes unit.
        subst_result = UpdateSubstitutionIfPure(*it, &subst);
      }
      // Remove the clause if:
      //  - the clause is true or;
      //  - the variable is temporary.
      if (is_true(*it) || (subst_result && map.count(*subst_result) > 0)) {
        it = clauses->erase(it);
      } else {
        ++it;
      }
    }
  } while (subst_result);
}

}  // namespace

// The main function of the TseitinCnfizer:
//  - It visits each node and introduce a Boolean variable `b` for
//    each subterm `f`, and keep the relation `b ⇔ f`.
//  - Then it cnfizes each `b ⇔ f` and make a conjunction of them.
vector<Formula> TseitinCnfizer::Convert(const Formula& f) {
  map_.clear();
  vector<Formula> ret{Visit(f)};
  for (auto const& p : map_) {
    Cnfize(p.first, p.second, &ret);
  }
  EliminatePureLiterals(map_, &ret);
  return ret;
}

Formula TseitinCnfizer::Visit(const Formula& f) {
  // TODO(soonho): use cache.
  return VisitFormula<Formula>(this, f);
}

Formula TseitinCnfizer::VisitFalse(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitTrue(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitVariable(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitEqualTo(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitNotEqualTo(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitGreaterThan(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitGreaterThanOrEqualTo(const Formula& f) {
  return f;
}
Formula TseitinCnfizer::VisitLessThan(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitLessThanOrEqualTo(const Formula& f) { return f; }
Formula TseitinCnfizer::VisitForall(const Formula& f) {
  // Given: f := ∀y. φ(x, y), this process CNFizes φ(x, y) and push the
  // universal quantifier over conjunctions:
  //
  //     = ∀y. (clause₁(x, y) ∧ ... ∧ clauseₙ(x, y))
  //     = (∀y. clause₁(x, y)) ∧ ... ∧ (∀y. clauseₙ(x, y))
  const Variables& quantified_variables{get_quantified_variables(f)};  // y
  const Formula& quantified_formula{get_quantified_formula(f)};  // φ(x, y)
  // clause₁(x, y) ∧ ... ∧ clauseₙ(x, y)
  const set<Formula> clauses{
      get_clauses(naive_cnfizer_.Convert(quantified_formula))};
  const set<Formula> new_clauses{
      ::dreal::map(clauses, [&quantified_variables](const Formula& clause) {
        DREAL_ASSERT(is_clause(clause));
        if (HaveIntersection(clause.GetFreeVariables(), quantified_variables)) {
          return forall(quantified_variables, clause);
        } else {
          return clause;
        }
      })};

  DREAL_ASSERT(!new_clauses.empty());
  if (new_clauses.size() == 1) {
    return *(new_clauses.begin());
  } else {
    static size_t id{0};
    const Variable bvar{string("forall") + to_string(id++),
                        Variable::Type::BOOLEAN};
    map_.emplace(bvar, make_conjunction(new_clauses));
    return Formula{bvar};
  }
}

Formula TseitinCnfizer::VisitConjunction(const Formula& f) {
  // Introduce a new Boolean variable, `bvar` for `f` and record the
  // relation `bvar ⇔ f`.
  static size_t id{0};
  const set<Formula> transformed_operands{::dreal::map(
      get_operands(f),
      [this](const Formula& formula) { return this->Visit(formula); })};
  const Variable bvar{string("conj") + to_string(id++),
                      Variable::Type::BOOLEAN};
  map_.emplace(bvar, make_conjunction(transformed_operands));
  return Formula{bvar};
}

Formula TseitinCnfizer::VisitDisjunction(const Formula& f) {
  static size_t id{0};
  const set<Formula>& transformed_operands{::dreal::map(
      get_operands(f),
      [this](const Formula& formula) { return this->Visit(formula); })};
  const Variable bvar{string("disj") + to_string(id++),
                      Variable::Type::BOOLEAN};
  map_.emplace(bvar, make_disjunction(transformed_operands));
  return Formula{bvar};
}

Formula TseitinCnfizer::VisitNegation(const Formula& f) {
  const Formula& operand{get_operand(f)};
  if (is_atomic(operand)) {
    return f;
  } else {
    const Variable bvar{"neg", Variable::Type::BOOLEAN};
    const Formula transformed_operand{Visit(operand)};
    map_.emplace(bvar, !transformed_operand);
    return Formula{bvar};
  }
}

namespace {
// Cnfizes b ⇔ f and adds it to @p clauses. It calls
// Cnfize<FormulaKind> using pattern-matching.
void Cnfize(const Variable& b, const Formula& f, vector<Formula>* clauses) {
  switch (f.get_kind()) {
    case FormulaKind::False:
      DREAL_UNREACHABLE();
    case FormulaKind::True:
      DREAL_UNREACHABLE();
    case FormulaKind::Var:
      DREAL_UNREACHABLE();
    case FormulaKind::Eq:
      DREAL_UNREACHABLE();
    case FormulaKind::Neq:
      DREAL_UNREACHABLE();
    case FormulaKind::Gt:
      DREAL_UNREACHABLE();
    case FormulaKind::Geq:
      DREAL_UNREACHABLE();
    case FormulaKind::Lt:
      DREAL_UNREACHABLE();
    case FormulaKind::Leq:
      DREAL_UNREACHABLE();
    case FormulaKind::Forall:
      DREAL_UNREACHABLE();
    case FormulaKind::And:
      return CnfizeConjunction(b, f, clauses);
    case FormulaKind::Or:
      return CnfizeDisjunction(b, f, clauses);
    case FormulaKind::Not:
      return CnfizeNegation(b, f, clauses);
  }
  DREAL_UNREACHABLE();
}

// Add f to clauses if f is not true.
void Add(const Formula& f, vector<Formula>* clauses) {
  if (!is_true(f)) {
    clauses->push_back(f);
  }
}

// Add f₁ ⇔ f₂ to clauses.
void AddIff(const Formula& f1, const Formula& f2, vector<Formula>* clauses) {
  Add(imply(f1, f2), clauses);
  Add(imply(f2, f1), clauses);
}

// Cnfize b ⇔ ¬b₁ using the following equalities and add to clauses:
//   b ⇔ ¬b₁
// = (b → ¬b₁) ∧ (¬b₁ → b)
// = (¬b ∨ ¬b₁) ∧ (b₁ ∨ b)   (✓CNF)
void CnfizeNegation(const Variable& b, const Formula& f,
                    vector<Formula>* clauses) {
  AddIff(Formula{b}, f, clauses);
}  // namespace

// Cnfize b ⇔ (b₁ ∧ ... ∧ bₙ) using the following equalities and add
// to clauses:
//   b ⇔ (b₁ ∧ ... ∧ bₙ)
// = (b → (b₁ ∧ ... ∧ bₙ)) ∧ ((b₁ ∧ ... ∧ bₙ) → b)
// = (¬b ∨ (b₁ ∧ ... ∧ bₙ)) ∧ (¬b₁ ∨ ... ∨ ¬bₙ ∨ b)
// = (¬b ∨ b₁) ∧ ... (¬b ∨ bₙ) ∧ (¬b₁ ∨ ... ∨ ¬bₙ ∨ b)   (✓CNF)
void CnfizeConjunction(const Variable& b, const Formula& f,
                       vector<Formula>* clauses) {
  // operands = {b₁, ..., bₙ}
  const set<Formula>& operands{get_operands(f)};
  // negated_operands = {¬b₁, ..., ¬bₙ}
  const set<Formula>& negated_operands{
      map(operands, [](const Formula& formula) { return !formula; })};
  Formula ret{Formula::True()};
  for (const Formula& b_i : operands) {
    Add(!b || b_i, clauses);
  }
  Add(make_disjunction(negated_operands) || b, clauses);
}

// Cnfize b ⇔ (b₁ ∨ ... ∨ bₙ) using the following equalities and add
// to clauses:
//   b ⇔ (b₁ ∨ ... ∨ bₙ)
// = (b → (b₁ ∨ ... ∨ bₙ)) ∧ ((b₁ ∨ ... ∨ bₙ) → b)
// = (¬b ∨ b₁ ∨ ... ∨ bₙ) ∧ ((¬b₁ ∧ ... ∧ ¬bₙ) ∨ b)
// = (¬b ∨ b₁ ∨ ... ∨ bₙ) ∧ (¬b₁ ∨ b) ∧ ... ∧ (¬bₙ ∨ b)   (✓CNF)
void CnfizeDisjunction(const Variable& b, const Formula& f,
                       vector<Formula>* clauses) {
  // negated_operands = {¬b₁, ..., ¬bₙ}
  const set<Formula>& negated_operands{
      map(get_operands(f), [](const Formula& formula) { return !formula; })};
  Add(!b || f, clauses);  // (¬b ∨ b₁ ∨ ... ∨ bₙ)
  for (const Formula& neg_b_i : negated_operands) {
    Add(neg_b_i || b, clauses);
  }
}
}  // namespace
}  // namespace dreal
