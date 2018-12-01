#pragma once

/// @file
/// This file contains abbreviated definitions for certain uses of
/// AutoDiffScalar that are commonly used in Drake.
/// @see also eigen_types.h

#include <type_traits>

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

#include "dreal/util/eigen_types.h"

namespace drake {

/// An autodiff variable with a dynamic number of partials.
using AutoDiffXd = Eigen::AutoDiffScalar<Eigen::VectorXd>;

/// An autodiff variable with `num_vars` partials.
template <int num_vars>
using AutoDiffd = Eigen::AutoDiffScalar<Eigen::Matrix<double, num_vars, 1> >;

/// A vector of `rows` autodiff variables, each with `num_vars` partials.
template <int num_vars, int rows>
using AutoDiffVecd = Eigen::Matrix<AutoDiffd<num_vars>, rows, 1>;

/// A dynamic-sized vector of autodiff variables, each with a dynamic-sized
/// vector of partials.
typedef AutoDiffVecd<Eigen::Dynamic, Eigen::Dynamic> AutoDiffVecXd;

}  // namespace drake
