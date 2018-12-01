/// @file
/// Overloads for STL mathematical operations on AutoDiffScalar.
///
/// Used via argument-dependent lookup (ADL). These functions appear
/// in the Eigen namespace so that ADL can automatically choose between
/// the STL version and the overloaded version to match the type of the
/// arguments. The proper use would be e.g.
///
/// \code{.cc}
///    void mymethod() {
///       using std::isinf;
///       isinf(myval);
///    }
/// \endcode{}
///
/// @note The if_then_else and cond functions for AutoDiffScalar are in
/// namespace drake because cond is defined in namespace drake in
/// "drake/common/cond.h" file.

#pragma once

#include <cmath>
#include <limits>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

namespace Eigen {

/// Overloads round to mimic std::round from <cmath>.
template <typename DerType>
double round(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::round;
  return round(x.value());
}

/// Overloads isinf to mimic std::isinf from <cmath>.
template <typename DerType>
bool isinf(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::isinf;
  return isinf(x.value());
}

/// Overloads isnan to mimic std::isnan from <cmath>.
template <typename DerType>
bool isnan(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::isnan;
  return isnan(x.value());
}

/// Overloads floor to mimic std::floor from <cmath>.
template <typename DerType>
double floor(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::floor;
  return floor(x.value());
}

/// Overloads ceil to mimic std::ceil from <cmath>.
template <typename DerType>
double ceil(const Eigen::AutoDiffScalar<DerType>& x) {
  using std::ceil;
  return ceil(x.value());
}

/// Overloads copysign from <cmath>.
template <typename DerType, typename T>
Eigen::AutoDiffScalar<DerType> copysign(const Eigen::AutoDiffScalar<DerType>& x,
                                        const T& y) {
  using std::isnan;
  if (isnan(x)) return (y >= 0) ? NAN : -NAN;
  if ((x < 0 && y >= 0) || (x >= 0 && y < 0))
    return -x;
  else
    return x;
}

/// Overloads copysign from <cmath>.
template <typename DerType>
double copysign(double x, const Eigen::AutoDiffScalar<DerType>& y) {
  using std::isnan;
  if (isnan(x)) return (y >= 0) ? NAN : -NAN;
  if ((x < 0 && y >= 0) || (x >= 0 && y < 0))
    return -x;
  else
    return x;
}

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)  // Eigen Version >= v3.3.0
#define DREAL_EIGEN_MAKE_AUTODIFFSCALAR(X, Y) Eigen::MakeAutoDiffScalar(X, Y)
#else
#define DREAL_EIGEN_MAKE_AUTODIFFSCALAR(X, Y) \
  { X, Y }
#endif

/// Overloads pow for an AutoDiffScalar base and exponent, implementing the
/// chain rule.
template <typename DerTypeA, typename DerTypeB>
Eigen::AutoDiffScalar<
    typename internal::remove_all<DerTypeA>::type::PlainObject>
pow(const Eigen::AutoDiffScalar<DerTypeA>& base,
    const Eigen::AutoDiffScalar<DerTypeB>& exponent) {
  // The two AutoDiffScalars being exponentiated must have the same matrix
  // type. This includes, but is not limited to, the same scalar type and
  // the same dimension.
  static_assert(
      std::is_same<
          typename internal::remove_all<DerTypeA>::type::PlainObject,
          typename internal::remove_all<DerTypeB>::type::PlainObject>::value,
      "The derivative types must match.");

  internal::make_coherent(base.derivatives(), exponent.derivatives());

  const auto& x = base.value();
  const auto& xgrad = base.derivatives();
  const auto& y = exponent.value();
  const auto& ygrad = exponent.derivatives();

  using std::log;
  using std::pow;
  const auto x_to_the_y = pow(x, y);
  if (ygrad.isZero(std::numeric_limits<double>::epsilon()) ||
      ygrad.size() == 0) {
    // The derivative only depends on ∂(x^y)/∂x -- this prevents undefined
    // behavior in the corner case where ∂(x^y)/∂y is infinite when x = 0,
    // despite ∂y/∂v being 0.
    return DREAL_EIGEN_MAKE_AUTODIFFSCALAR(x_to_the_y,
                                           y * pow(x, y - 1) * xgrad);
  }
  return DREAL_EIGEN_MAKE_AUTODIFFSCALAR(
      // The value is x ^ y.
      x_to_the_y,
      // The multivariable chain rule states:
      // df/dv_i = (∂f/∂x * dx/dv_i) + (∂f/∂y * dy/dv_i)
      // ∂f/∂x is y*x^(y-1)
      y * pow(x, y - 1) * xgrad +
          // ∂f/∂y is (x^y)*ln(x)
          x_to_the_y * log(x) * ygrad);
}

#undef DREAL_EIGEN_MAKE_AUTODIFFSCALAR

#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
#define EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(FUNC, CODE)                      \
  template <typename DerType>                                                \
  inline const Eigen::AutoDiffScalar<EIGEN_EXPR_BINARYOP_SCALAR_RETURN_TYPE( \
      typename Eigen::internal::remove_all<DerType>::type,                   \
      typename Eigen::internal::traits<                                      \
          typename Eigen::internal::remove_all<DerType>::type>::Scalar,      \
      product)>                                                              \
  FUNC(const Eigen::AutoDiffScalar<DerType>& x) {                            \
    EIGEN_UNUSED typedef typename Eigen::internal::traits<                   \
        typename Eigen::internal::remove_all<DerType>::type>::Scalar Scalar; \
    CODE;                                                                    \
  }
EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(
    atan, using std::atan; return Eigen::MakeAutoDiffScalar(
        atan(x.value()),
        x.derivatives() * (Scalar(1) / (x.value() * x.value())));)
#else
#define EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(FUNC, CODE)                      \
  template <typename DerType>                                                \
  inline const Eigen::AutoDiffScalar<Eigen::CwiseUnaryOp<                    \
      Eigen::internal::scalar_multiple_op<typename Eigen::internal::traits<  \
          typename Eigen::internal::remove_all<DerType>::type>::Scalar>,     \
      const typename Eigen::internal::remove_all<DerType>::type>>            \
  FUNC(const Eigen::AutoDiffScalar<DerType>& x) {                            \
    typedef typename Eigen::internal::traits<                                \
        typename Eigen::internal::remove_all<DerType>::type>::Scalar Scalar; \
    typedef AutoDiffScalar<CwiseUnaryOp<                                     \
        Eigen::internal::scalar_multiple_op<Scalar>,                         \
        const typename Eigen::internal::remove_all<DerType>::type>>          \
        ReturnType;                                                          \
    CODE;                                                                    \
  }

EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(
    atan, using std::atan;
    return ReturnType(atan(x.value()),
                      x.derivatives() * (Scalar(1) / (x.value() * x.value())));)

EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(
    tanh, using std::cosh; using std::tanh;
    return ReturnType(tanh(x.value()),
                      x.derivatives() *
                          (Scalar(1) / numext::abs2(cosh(x.value()))));)

EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(
    sinh, using std::sinh; using std::cosh;
    return ReturnType(sinh(x.value()), x.derivatives() * cosh(x.value()));)

EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY(
    cosh, using std::sinh; using std::cosh;
    return ReturnType(cosh(x.value()), x.derivatives() * sinh(x.value()));)

template <typename DerType>
inline AutoDiffScalar<
    typename Eigen::internal::remove_all<DerType>::type::PlainObject>(min)(
    const AutoDiffScalar<DerType>& x, const AutoDiffScalar<DerType>& y) {
  return (x.value() < y.value() ? x : y);
}
template <typename DerType>
inline AutoDiffScalar<
    typename Eigen::internal::remove_all<DerType>::type::PlainObject>(max)(
    const AutoDiffScalar<DerType>& x, const AutoDiffScalar<DerType>& y) {
  return (x.value() >= y.value() ? x : y);
}
#endif

#undef EIGEN_AUTODIFF_DECLARE_GLOBAL_UNARY

}  // namespace Eigen
