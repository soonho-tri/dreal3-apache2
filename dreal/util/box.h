#pragma once

#include <iostream>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "./ibex.h"

#include "dreal/symbolic/symbolic.h"

namespace dreal {

class BoxCell;

/// Represents a n-dimensional interval vector. This is a wrapper of
/// ibex::IntervalVector.
class Box {
 public:
  using Interval = ibex::Interval;
  using IntervalVector = ibex::IntervalVector;

  /// Constructs a zero-dimensional box.
  Box();

  /// Constructs a box from @p variables.
  explicit Box(const std::vector<Variable>& variables);

  /// Copy constructor.
  Box(const Box& b);

  /// Move constructor.
  Box(Box&& b) noexcept;

  /// Copy assign operator.
  Box& operator=(const Box& b);

  /// Move assign operator.
  Box& operator=(Box&& b) noexcept;

  /// Destructor.
  ~Box();

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
  Box& InplaceUnion(const Box& b);

 private:
  /// Added ----
  explicit Box(BoxCell* ptr);

  BoxCell* ptr_{nullptr};

  /// Added ----

  friend std::ostream& operator<<(std::ostream& os, const Box& box);
  friend bool operator==(const Box& b1, const Box& b2);
  friend BoxCell;
};

std::ostream& operator<<(std::ostream& os, const Box& box);

bool operator==(const Box& b1, const Box& b2);

bool operator!=(const Box& b1, const Box& b2);

std::ostream& DisplayDiff(std::ostream& os,
                          const std::vector<Variable>& variables,
                          const Box::IntervalVector& old_iv,
                          const Box::IntervalVector& new_iv);

}  // namespace dreal
