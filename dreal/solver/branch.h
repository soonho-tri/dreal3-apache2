#pragma once

#include <tuple>
#include <utility>

#include "dreal/util/box.h"
#include "dreal/util/optional.h"

namespace dreal {

/// Finds the dimension with the maximum diameter in a @p box. It only
/// consider the dimensions enabled in @p bitset.
///
/// @returns a pair of (max dimension, variable index).
std::pair<double, int> FindMaxDiam(const Box& box, const ibex::BitSet& bitset);

/// Partitions @p box into two sub-boxes. It traverses only the
/// variables enabled by @p bitset, to find a branching dimension.
///
/// @returns If it finds a branching dimension then return a tuple<Box, Box, int
/// (branching dimension)>. Otherwise, it returns nullopt.
optional<std::tuple<Box, Box, int>> Branch(const Box& box,
                                           const ibex::BitSet& bitset);

}  // namespace dreal
