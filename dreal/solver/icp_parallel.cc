#include "dreal/solver/icp_parallel.h"

#include <atomic>
#include <cfenv>
#include <thread>
#include <tuple>
#include <utility>

#include "dreal/solver/branch.h"
#include "dreal/solver/icp_stat.h"
#include "dreal/util/assert.h"
#include "dreal/util/cds.h"
#include "dreal/util/interrupt.h"
#include "dreal/util/logging.h"

using std::atomic;
using std::atomic_flag;
using std::pair;
using std::vector;

namespace dreal {

namespace {

// Returns -1 if it detects that the interval vector is non-bisectable.
int FindMaxDiamIdx(const Box::IntervalVector& iv) {
  double max_diam{0.0};
  int max_diam_idx{-1};
  for (int i{0}; i < iv.size(); ++i) {
    const Box::Interval& iv_i{iv[i]};
    const double diam_i{iv_i.diam()};
    if (diam_i > max_diam && iv_i.is_bisectable()) {
      max_diam = diam_i;
      max_diam_idx = i;
    }
  }
  return max_diam_idx;
}

vector<Box::IntervalVector> DoubleUp(
    const vector<Box::IntervalVector>& interval_vectors, const int n) {
  DREAL_ASSERT(interval_vectors.size() <= static_cast<unsigned>(n));
  vector<Box::IntervalVector> ret;
  ret.reserve(n);
  vector<Box::IntervalVector>::size_type i{0};
  for (; i < n - interval_vectors.size() && i < interval_vectors.size(); ++i) {
    const Box::IntervalVector& iv{interval_vectors[i]};
    const int max_diam_idx{FindMaxDiamIdx(iv)};
    if (max_diam_idx >= 0) {
      const auto& bisect_result = iv.bisect(max_diam_idx);
      ret.push_back(bisect_result.first);
      ret.push_back(bisect_result.second);
    }
  }
  for (; i < interval_vectors.size(); ++i) {
    ret.push_back(interval_vectors[i]);
  }
  return ret;
}

vector<Box::IntervalVector> FillUp(const Box::IntervalVector& iv, int n) {
  vector<Box::IntervalVector> ret{iv};
  while (ret.size() < static_cast<unsigned>(n)) {
    vector<Box::IntervalVector> new_ones{DoubleUp(ret, n)};
    if (new_ones.size() == ret.size()) {
      break;
    } else {
      ret = new_ones;
    }
  }
  return ret;
}

bool ParallelBranch(const ibex::BitSet& bitset, const bool stack_left_box_first,
                    const int number_of_jobs, Box* const box,
                    vector<Box::IntervalVector>* const local_stack,
                    Stack<Box::IntervalVector>* const global_stack,
                    atomic<int>* const number_of_boxes,
                    atomic<int>* const global_stack_size) {
  const pair<double, int> max_diam_and_idx{FindMaxDiam(*box, bitset)};
  const int branching_point{max_diam_and_idx.second};
  if (branching_point >= 0) {
    const auto boxes = box->interval_vector().bisect(branching_point);
    const Box::IntervalVector* iv1_ptr{nullptr};
    const Box::IntervalVector* iv2_ptr{nullptr};
    if (stack_left_box_first) {
      iv1_ptr = &boxes.first;
      iv2_ptr = &boxes.second;
    } else {
      iv2_ptr = &boxes.first;
      iv1_ptr = &boxes.second;
    }
    const Box::IntervalVector& iv1{*iv1_ptr};
    const Box::IntervalVector& iv2{*iv2_ptr};

    number_of_boxes->fetch_add(1, std::memory_order_relaxed);
    // TODO(soonho): FIXME. Decision #1: when to add to the global stack or a
    // local_stack?
    if (global_stack_size->load(std::memory_order_acquire) < number_of_jobs) {
      global_stack_size->fetch_add(1, std::memory_order_relaxed);
      global_stack->push(iv1);
    } else {
      local_stack->push_back(iv1);
    }
    box->mutable_interval_vector() = iv2;
    return true;
  }
  // Fail to find a branching point.
  return false;
}

void Worker(const Contractor& contractor, const Config& config,
            const vector<FormulaEvaluator>& formula_evaluators, const int id,
            const bool main_thread,
            Stack<Box::IntervalVector>* const global_stack,
            ContractorStatus* const cs, atomic<int>* const found_delta_sat,
            atomic<int>* const number_of_boxes,
            atomic<int>* const global_stack_size, IcpStat* const stat) {
  thread_local vector<Box::IntervalVector> local_stack;
  local_stack.clear();
  local_stack.reserve(100);

  CdsScopeGuard cds_scope_guard(!main_thread);

  bool stack_left_box_first{config.stack_left_box_first()};

  // `current_box` always points to the box in the contractor status
  // as a mutable reference.
  Box& current_box{cs->mutable_box()};
  Box::IntervalVector& current_iv{current_box.mutable_interval_vector()};

  bool need_to_pop{true};

  while ((*found_delta_sat == -1) &&
         (!local_stack.empty() ||
          number_of_boxes->load(std::memory_order_acquire) > 0)) {
    // Note that 'DREAL_CHECK_INTERRUPT' is only defined in setup.py,
    // when we build dReal python package.
#ifdef DREAL_CHECK_INTERRUPT
    if (g_interrupted) {
      DREAL_LOG_DEBUG("KeyboardInterrupt(SIGINT) Detected.");
      throw std::runtime_error("KeyboardInterrupt(SIGINT) Detected.");
    }
#endif

    // 1. Pick a box from local and global stack if needed.
    //  A) First check the local stack.
    //  B) If the local stack is empty, get a box from the global stack.
    /// C) If there are nothing, spin.
    if (need_to_pop) {
      if (!local_stack.empty()) {
        current_iv = local_stack.back();
        local_stack.pop_back();
      } else if (!global_stack->pop(current_iv)) {
        // DREAL_LOG_DEBUG("IcpParallel::Worker() NO BOX.");
        continue;
      } else {
        global_stack_size->fetch_sub(1, std::memory_order_acq_rel);
      }
    }
    need_to_pop = true;

    // Spill (Load Balancing): Move an item from the local stack to the global
    // stack.
    while (!local_stack.empty() && global_stack->empty()) {
      global_stack_size->fetch_add(1, std::memory_order_relaxed);
      global_stack->push(local_stack.back());
      local_stack.pop_back();
    }

    // Populating the global stack if there are not enough boxes on it.
    if (global_stack->empty()) {
      bool first_one = true;
      for (const Box::IntervalVector& iv :
           FillUp(current_box.interval_vector(), config.number_of_jobs())) {
        if (first_one) {
          // We handle the first iv immediately.
          current_iv = iv;
          first_one = false;
        } else {
          // The rest of the boxes goes to the global stack.
          number_of_boxes->fetch_add(1, std::memory_order_relaxed);
          global_stack_size->fetch_add(1, std::memory_order_relaxed);
          global_stack->push(iv);
        }
      }
    }

    // 2. Prune the current box.

    // DREAL_LOG_TRACE("IcpParallel::Worker() Current Box:\n{}", current_box);
    contractor.Prune(cs);

    // stat->increase_prune();
    // DREAL_LOG_TRACE(
    //     "IcpParallel::Worker() After pruning, the current box =\n{}",
    //     current_box);

    if (current_box.empty()) {
      // 3.1. The box is empty after pruning.
      number_of_boxes->fetch_sub(1, std::memory_order_acq_rel);
      // DREAL_LOG_DEBUG("IcpParallel::Worker() Box is empty after pruning");
      continue;
    }

    // 3.2. The box is non-empty. Check if the box is still feasible
    // under evaluation and it's small enough.
    const optional<ibex::BitSet> evaluation_result{
        EvaluateBox(formula_evaluators, current_box, config.precision(), cs)};
    if (!evaluation_result) {
      // 3.2.1. We detect that the current box is not a feasible solution.
      number_of_boxes->fetch_sub(1, std::memory_order_acq_rel);
      DREAL_LOG_DEBUG(
          "IcpParallel::Worker() Detect that the current box is not feasible "
          "by evaluation:\n{}",
          current_box);
      continue;
    }
    if (evaluation_result->empty()) {
      // 3.2.2. delta - SAT: We find a box which is smaller enough.
      DREAL_LOG_DEBUG("IcpParallel::Worker() Found a delta-box:\n{}",
                      current_box);
      *found_delta_sat = id;
      return;
    }

    // 3.2.3. This box is bigger than delta. Need branching.
    if (!ParallelBranch(*evaluation_result, stack_left_box_first,
                        config.number_of_jobs(), &current_box, &local_stack,
                        global_stack, number_of_boxes, global_stack_size)) {
      DREAL_LOG_DEBUG(
          "IcpParallel::Worker() Found that the current box is not "
          "satisfying "
          "delta-condition but it's not bisectable.:\n{}",
          current_box);
      *found_delta_sat = id;
      return;
    }

    need_to_pop = false;

    // We alternate between adding-the-left-box-first policy and
    // adding-the-right-box-first policy.
    stack_left_box_first = !stack_left_box_first;
    // stat->increase_branch();
  }
}
}  // namespace

IcpParallel::IcpParallel(const Config& config)
    : Icp{config}, pool_{static_cast<size_t>(config.number_of_jobs())} {
  std::cerr << "IcpParallel is created.\n";
  status_vector_.reserve(config.number_of_jobs());
}

bool IcpParallel::CheckSat(const Contractor& contractor,
                           const vector<FormulaEvaluator>& formula_evaluators,
                           ContractorStatus* const cs) {
  DREAL_LOG_CRITICAL("IcpParallel::CheckSat()");
  atomic<int> found_delta_sat{-1};
  static CdsInit cds_init{
      true /* main thread is using lock-free containers. */};
  Stack<Box::IntervalVector> global_stack;
  const int number_of_jobs = config().number_of_jobs();
  const int number_of_initial_boxes = number_of_jobs;

  // Set up the global stack.
  for (const auto& iv :
       FillUp(cs->box().interval_vector(), number_of_initial_boxes)) {
    global_stack.push(iv);
  }
  atomic<int> number_of_boxes{number_of_initial_boxes};
  atomic<int> global_stack_size{number_of_initial_boxes};

  static IcpStat stat{DREAL_LOG_INFO_ENABLED};

  status_vector_.clear();
  for (int i = 0; i < number_of_jobs; ++i) {
    status_vector_.push_back(*cs);
  }

  for (int i = 0; i < number_of_jobs - 1; ++i) {
    status_vector_.push_back(*cs);
    pool_.enqueue(Worker, contractor, config(), formula_evaluators, i,
                  false /* not main thread */, &global_stack,
                  &status_vector_[i], &found_delta_sat, &number_of_boxes,
                  &global_stack_size, &stat);
  }

  const int last_index{number_of_jobs - 1};
  Worker(contractor, config(), formula_evaluators, last_index,
         true /* main thread */, &global_stack, &status_vector_[last_index],
         &found_delta_sat, &number_of_boxes, &global_stack_size, &stat);

  // Post-processing: Join all the contractor statuses.
  for (const auto& cs_i : status_vector_) {
    cs->InplaceJoin(cs_i);
  }

  if (found_delta_sat >= 0) {
    cs->mutable_box() = status_vector_[found_delta_sat].box();
    return true;
  } else {
    cs->mutable_box().set_empty();
    return false;
  }
}
}  // namespace dreal
