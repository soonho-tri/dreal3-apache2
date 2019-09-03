#include "dreal/util/box.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "dreal/util/assert.h"
#include "dreal/util/box_cell.h"
#include "dreal/util/exception.h"
#include "dreal/util/logging.h"
#include "dreal/util/math.h"

using std::ceil;
using std::equal;
using std::find_if;
using std::floor;
using std::make_pair;
using std::make_shared;
using std::numeric_limits;
using std::ostream;
using std::pair;
using std::unordered_map;
using std::vector;

namespace dreal {

Box::Box() : Box{new BoxCell{}} {}

Box::Box(const Box& b) : Box{b.ptr_} {
  assert(ptr_ != nullptr);
  ptr_->increase_rc();
}

Box::Box(Box&& b) noexcept : Box{b.ptr_} {
  assert(ptr_ != nullptr);
  b.ptr_ = nullptr;
}

Box& Box::operator=(const Box& b) { return *this = Box{b}; }

Box& Box::operator=(Box&& b) noexcept {
  assert(b.ptr_ != nullptr);
  if (ptr_) {
    ptr_->decrease_rc();
  }
  ptr_ = b.ptr_;
  b.ptr_ = nullptr;
  return *this;
}

/// Destructor.
Box::~Box() {
  if (ptr_) {
    ptr_->decrease_rc();
  }
}

Box::Box(const vector<Variable>& variables) : Box{new BoxCell{variables}} {}

Box::Box(BoxCell* ptr) : ptr_{ptr} {
  assert(ptr_ != nullptr);
  ptr_->increase_rc();
}

void Box::Add(const Variable& v) { ptr_->Add(v); }

void Box::Add(const Variable& v, const double lb, const double ub) {
  ptr_->Add(v, lb, ub);
}

bool Box::empty() const { return ptr_->empty(); }

void Box::set_empty() { ptr_->set_empty(); }

int Box::size() const { return ptr_->size(); }

Box::Interval& Box::operator[](const int i) { return (*ptr_)[i]; }
Box::Interval& Box::operator[](const Variable& var) { return (*ptr_)[var]; }
const Box::Interval& Box::operator[](const int i) const { return (*ptr_)[i]; }
const Box::Interval& Box::operator[](const Variable& var) const {
  return (*ptr_)[var];
}

const vector<Variable>& Box::variables() const { return ptr_->variables(); }

const Variable& Box::variable(const int i) const { return ptr_->variable(i); }

bool Box::has_variable(const Variable& var) const {
  return ptr_->has_variable(var);
}

int Box::index(const Variable& var) const { return ptr_->index(var); }

const Box::IntervalVector& Box::interval_vector() const {
  return ptr_->interval_vector();
}
Box::IntervalVector& Box::mutable_interval_vector() {
  return ptr_->mutable_interval_vector();
}

pair<double, int> Box::MaxDiam() const { return ptr_->MaxDiam(); }

pair<Box, Box> Box::bisect(const int i) const { return ptr_->bisect(i); }

pair<Box, Box> Box::bisect(const Variable& var) const {
  return ptr_->bisect(var);
}

Box& Box::InplaceUnion(const Box& b) {
  ptr_->InplaceUnion(*(b.ptr_));
  return *this;
}

ostream& operator<<(ostream& os, const Box& box) { return os << *(box.ptr_); }

bool operator==(const Box& b1, const Box& b2) {
  return *(b1.ptr_) == *(b2.ptr_);
}

bool operator!=(const Box& b1, const Box& b2) { return !(b1 == b2); }

// ostream& DisplayDiff(ostream& os, const vector<Variable>& variables,
//                      const Box::IntervalVector& old_iv,
//                      const Box::IntervalVector& new_iv) {

ostream& DisplayDiff(ostream&, const vector<Variable>&,
                     const Box::IntervalVector&, const Box::IntervalVector&) {
  // return ptr_->DisplayDiff(os, variables, old_iv, new_iv);
  // TODO(soonho): fixme
  throw 1;
}

}  // namespace dreal
