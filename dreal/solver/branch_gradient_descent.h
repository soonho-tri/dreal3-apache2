#pragma once

#include <utility>
#include <vector>

#include "dreal/solver/config.h"
#include "dreal/solver/formula_evaluator.h"
#include "dreal/util/box.h"

namespace dreal {

bool BranchGradientDescent(const VectorX<Expression>& constraints,
                           const Config& config,
                           const ibex::BitSet& branching_candidates, Box* box,
                           std::vector<std::pair<Box, int>>* stack);

}  // namespace dreal
