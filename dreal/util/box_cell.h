#pragma once

#include <atomic>
#include <memory>
#include <unordered_map>
#include <utility>

#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {

class BoxCell {
 public:
  using Interval = Box::Interval;
  using IntervalVector = Box::IntervalVector;

  /// Constructs a zero-dimensional box.
  BoxCell();

  /// Constructs a box from @p variables.
  explicit BoxCell(const std::vector<Variable>& variables);

  /// Default copy constructor.
  BoxCell(const BoxCell&) = default;

  /// Default move constructor.
  BoxCell(BoxCell&&) = default;

  /// Default copy assign operator.
  BoxCell& operator=(const BoxCell&) = default;

  /// Default move assign operator.
  BoxCell& operator=(BoxCell&&) = default;

  /// Default destructor.
  ~BoxCell() = default;

  /// Adds @p v to the box.
  void Add(const Variable& v);

  /// Adds @p v to the box and sets its domain using @p lb and @p ub.
  void Add(const Variable& v, double lb, double ub);

  /// Checks if this box is empty.
  bool empty() const;

  /// Make this box empty.
  void set_empty();

  /// Returns the size of the box.
  int size() const;

  /// Returns @p i -th interval in the box.
  Interval& operator[](int i);

  /// Returns an interval associated with @p var.
  Interval& operator[](const Variable& var);

  /// Returns @p i -th interval in the box.
  const Interval& operator[](int i) const;

  /// Returns an interval associated with @p var.
  const Interval& operator[](const Variable& var) const;

  /// Returns the variables in the box.
  const std::vector<Variable>& variables() const;

  /// Returns i-th variable in the box.
  const Variable& variable(int i) const;

  /// Checks if this box has @p var.
  bool has_variable(const Variable& var) const;

  /// Returns the interval vector of the box.
  const IntervalVector& interval_vector() const;

  /// Returns the interval vector of the box.
  IntervalVector& mutable_interval_vector();

  /// Returns the index associated with @p var.
  int index(const Variable& var) const;

  /// Returns the max diameter of the box and the associated index .
  std::pair<double, int> MaxDiam() const;

  /// Bisects the box at @p i -th dimension.
  /// @throws std::runtime if @p i -th dimension is not bisectable.
  std::pair<Box, Box> bisect(int i) const;

  /// Bisects the box at @p the dimension represented by @p var.
  /// @throws std::runtime if @p i -th dimension is not bisectable.
  std::pair<Box, Box> bisect(const Variable& var) const;

  /// Updates the current box by taking union with @p b.
  ///
  /// @pre variables() == b.variables().
  void InplaceUnion(const BoxCell& b);

 private:
  /// Bisects the box at @p i -th dimension.
  /// @pre i-th variable is bisectable.
  /// @pre i-th variable is of integer type.
  std::pair<Box, Box> bisect_int(int i) const;

  /// Bisects the box at @p i -th dimension.
  /// @pre i-th variable is bisectable.
  /// @pre i-th variable is of continuous type.
  std::pair<Box, Box> bisect_continuous(int i) const;

  std::shared_ptr<std::vector<Variable>> variables_;

  Box::IntervalVector values_;

  std::shared_ptr<std::unordered_map<Variable, int, hash_value<Variable>>>
      var_to_idx_;

  std::shared_ptr<std::unordered_map<int, Variable>> idx_to_var_;

  // Reference counter.
  mutable std::atomic<unsigned> rc_{0};
  void increase_rc() const {
    atomic_fetch_add_explicit(&rc_, 1U, std::memory_order_relaxed);
  }
  void decrease_rc() const {
    if (atomic_fetch_sub_explicit(&rc_, 1U, std::memory_order_acq_rel) == 1U) {
      delete this;
    }
  }

  friend std::ostream& operator<<(std::ostream& os, const BoxCell& box);
  friend bool operator==(const BoxCell& b1, const BoxCell& b2);
};

std::ostream& operator<<(std::ostream& os, const BoxCell& box);

bool operator==(const BoxCell& b1, const BoxCell& b2);

}  // namespace dreal
