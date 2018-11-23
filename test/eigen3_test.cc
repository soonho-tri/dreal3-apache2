// This example is taken from
// https://github.com/edrumwri/drake/blob/autodiff_doc/drake/doc/autodiff_intro/autodiff.tex

#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <gtest/gtest.h>
#include <unsupported/Eigen/AutoDiff>

namespace {

template <class T>
T sin_T(const T& a) {
  using std::sin;
  return sin(a);
}

using std::cout;
using std::endl;

GTEST_TEST(Eigen3, Autodiff) {
  typedef Eigen::AutoDiffScalar<Eigen::Matrix<double, 1, 1>> AScalar;
  AScalar x = 3 * M_PI / 4.0;
  x.derivatives()(0) = 1;

  EXPECT_NEAR(sin_T(x).value(), 0.70710678118654757, 1e-10);
  EXPECT_NEAR(sin_T(x).derivatives()(0), -0.70710678118654746, 1e-10);
}

}  // namespace
