#pragma once

#include <utility>
#include <vector>

#include "dreal/util/box.h"

namespace dreal {
bool BranchMaxDiam(const Box& box, const ibex::BitSet& branching_candidates,
                   const bool stack_left_box_first,
                   std::vector<std::pair<Box, int>>* stack);
}  // namespace dreal
