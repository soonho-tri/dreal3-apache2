#include <cmath>

#include "dreal/solver/branch_gradient_descent.h"
#include "dreal/util/eigen.h"
#include "dreal/util/evaluation_visitor.h"

namespace dreal {

using std::pair;
using std::vector;

namespace {

/// Returns the center point of box @p b.
Eigen::VectorXd SamplePoint(const Box& b) {
  Eigen::VectorXd c{b.size()};
  for (int i{0}; i < b.size(); ++i) {
    if (b[i].lb() == -INFINITY) {
      if (b[i].ub() == INFINITY) {
        c[i] = 0.0;
      } else {
        c[i] = b[i].ub();
      }
    } else if (b[i].ub() == INFINITY) {
      c[i] = b[i].lb();
    } else {
      c[i] = b[i].mid();
    }
  }
  return c;
}

// Set @p box to the point @p p.
void SetBox(const Eigen::VectorXd& p, Box* const box) {
  for (int i{0}; i < box->size(); ++i) {
    (*box)[i] = p[i];
  }
}

/// Clamps *this by the box b and returns a reference to *this.
void Clamp(const Box& b, double* const p) {
  for (int i{0}; i < b.size(); ++i) {
    double& value{p[i]};
    const Box::Interval& iv{b[i]};
    if (value < iv.lb()) {
      value = iv.lb();
    } else if (iv.ub() < value) {
      value = iv.ub();
    }
  }
}

/// Initialize q with p.
void InitializeAutodiff(const Eigen::VectorXd& p,
                        VectorX<Eigen::AutoDiffScalar<Eigen::VectorXd>>* q) {
  const auto size_of_p{p.size()};
  for (int i{0}; i < size_of_p; ++i) {
    (*q)[i] = p[i];
    (*q)(i).derivatives() = Eigen::VectorXd::Unit(size_of_p, i);
  }
}

bool IncludeNaN(const Eigen::VectorXd& v) { return v.array().isNaN().any(); }

}  // namespace

// P = Sample(box)  # For now, just take the center point.
// α = 0.001        # parameter.
//
// # Goal: Move P to minimize errors
// for i = 1 to MaxIter:
//     stop = true
//
//     for f in constraints:
//         error = f(P)
//         if error ≥ 0:
//             P' -= α * ∇f(P) * error
//             error' = f(P')
//             if error' < error:
//                 P = P'
//                 stop = false
//
//     if stop:
//         # It turns out that no constraint can move P.
//         # We stop here.
//         break
bool BranchGradientDescent(const VectorX<Expression>& constraints,
                           const Config& config,
                           const ibex::BitSet& branching_candidates, Box* box,
                           vector<pair<Box, int>>* stack) {
  // Constants
  const int kMaxIter{config.branch_gradient_descent_max_iter_.get()};
  const double kQuickFactor{config.branch_gradient_descent_quick_factor_.get()};
  const double kBrakeFactor{config.branch_gradient_descent_brake_factor_.get()};
  const double kInitAlpha{config.branch_gradient_descent_alpha_.get()};
  Eigen::VectorXd alpha{
      Eigen::VectorXd::Constant(constraints.size(), kInitAlpha)};
  Eigen::VectorXd init_p{SamplePoint(*box)};
  Eigen::VectorXd p{init_p};

  VectorX<Eigen::AutoDiffScalar<Eigen::VectorXd>> q(p.size());
  const EvaluationVisitor evaluation_visitor{box->variables()};
  bool stop{false};
  for (int i{0}; i < kMaxIter && !stop; ++i) {
    stop = true;
    for (VectorX<Expression>::Index j{0}; j < constraints.size(); ++j) {
      const Expression& e_j{constraints[j]};
      InitializeAutodiff(p, &q);  // Initialize q as p.
      Eigen::AutoDiffScalar<Eigen::VectorXd> eval =
          evaluation_visitor.Evaluate<Eigen::AutoDiffScalar<Eigen::VectorXd>>(
              e_j, q);
      const double current_error{eval.value()};
      if (current_error > config.precision()) {
        const auto derivatives{eval.derivatives()};
        if (IncludeNaN(derivatives)) {
          continue;
        }
        const auto p_next = p - (derivatives * alpha[j] * current_error);
        const double new_error =
            evaluation_visitor.Evaluate<double>(e_j, p_next);
        if (new_error < current_error) {
          stop = false;
          p = p_next;
          Clamp(*box, p.data());
          alpha[j] *= kQuickFactor;
        } else {
          alpha[j] *= kBrakeFactor;
        }
      }
    }
  }

  const Eigen::VectorXd final_error{
      evaluation_visitor.Evaluate<double>(constraints, p)};
  if ((final_error.array() <= config.precision()).all()) {
    SetBox(p, box);
    return true;
  }

  double max_shift = -999;
  int branching_point = -1;
  double shift{0.0};
  for (int i{0}, idx = branching_candidates.min();
       i < branching_candidates.size();
       ++i, idx = branching_candidates.next(idx)) {
    DREAL_ASSERT(idx < box->size());
    DREAL_ASSERT((*box)[idx].is_bisectable());
    shift = p[idx] - init_p[idx];
    const double abs_shift{std::abs(shift)};
    if (abs_shift > max_shift) {
      branching_point = idx;
      max_shift = abs_shift;
    }
  }
  if (branching_point == -1) {
    return true;
  } else {
    const pair<Box, Box> bisected_boxes{box->bisect(branching_point)};
    if (shift > 0.0) {
      stack->emplace_back(bisected_boxes.first, branching_point);
      stack->emplace_back(bisected_boxes.second, branching_point);
    } else {
      stack->emplace_back(bisected_boxes.second, branching_point);
      stack->emplace_back(bisected_boxes.first, branching_point);
    }
  }
  return false;
}
}  // namespace dreal
