#include "dreal/contractor/contractor_ibex_fwdbwd.h"

#include <atomic>
#include <sstream>
#include <utility>

#include "dreal/util/assert.h"
#include "dreal/util/logging.h"
#include "dreal/util/math.h"
#include "dreal/util/stat.h"
#include "dreal/util/timer.h"

using std::cout;
using std::make_unique;
using std::ostream;
using std::ostringstream;

namespace dreal {

namespace {
class ContractorIbexFwdbwdStat : public Stat {
 public:
  explicit ContractorIbexFwdbwdStat(const bool enabled) : Stat{enabled} {};
  ContractorIbexFwdbwdStat(const ContractorIbexFwdbwdStat&) = delete;
  ContractorIbexFwdbwdStat(ContractorIbexFwdbwdStat&&) = delete;
  ContractorIbexFwdbwdStat& operator=(const ContractorIbexFwdbwdStat&) = delete;
  ContractorIbexFwdbwdStat& operator=(ContractorIbexFwdbwdStat&&) = delete;
  ~ContractorIbexFwdbwdStat() override {
    if (enabled()) {
      using fmt::print;
      print(cout, "{:<45} @ {:<20} = {:>15}\n",
            "Total # of ibex-fwdbwd Pruning", "Pruning level", num_pruning_);
      print(cout, "{:<45} @ {:<20} = {:>15}\n",
            "Total # of ibex-fwdbwd Pruning (zero-effect)", "Pruning level",
            num_zero_effect_pruning_);
    }
  }

  void increase_zero_effect_pruning() { increase(&num_zero_effect_pruning_); }

  void increase_pruning() { increase(&num_pruning_); }

 private:
  std::atomic<int> num_zero_effect_pruning_{0};
  std::atomic<int> num_pruning_{0};
};
}  // namespace

//---------------------------------------
// Implementation of ContractorIbexFwdbwd
//---------------------------------------
ContractorIbexFwdbwd::ContractorIbexFwdbwd(Formula f, const Box& box,
                                           const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_FWDBWD,
                     ibex::BitSet::empty(box.size()), config},
      f_{std::move(f)},
      ibex_converters_(),
      expr_ctr_map_(config.number_of_jobs()),
      ctc_map_(config.number_of_jobs()) {
  ibex_converters_.reserve(config.number_of_jobs());
  ibex::CtcFwdBwd* ctc = GetCtc(box);
  if (ctc) {
    ibex::BitSet& input{mutable_input()};
    for (const Variable& var : f_.GetFreeVariables()) {
      input.add(box.index(var));
    }
  }
}

ContractorIbexFwdbwd::~ContractorIbexFwdbwd() {
  for (const auto& pair : expr_ctr_map_) {
    const std::thread::id& id{pair.first};
    const ibex::ExprCtr* expr_ctr{pair.second};

    auto ctc = ctc_map_.find(id);
    auto num_ctr = &(ctc->ctr);
    delete ctc;
    delete num_ctr;
    delete expr_ctr;
  }
  for (const auto& ibex_converter : ibex_converters_) {
    delete ibex_converter;
  }
}

ibex::CtcFwdBwd* ContractorIbexFwdbwd::GetCtc(const Box& box) const {
  const thread_local auto id = std::this_thread::get_id();

  if (ctc_map_.contains(id)) {
    return ctc_map_.find(id);
  } else {
    auto ibex_converter = make_unique<IbexConverter>(box);
    auto expr_ctr{ibex_converter->Convert(f_)};

    // Build num_ctr and ctc_.
    if (expr_ctr) {
      ibex::NumConstraint* num_ctr{nullptr};
      {
        std::lock_guard<std::mutex> lock{m_};
        expr_ctr_map_.emplace(id, expr_ctr);
        num_ctr =
            new ibex::NumConstraint{ibex_converter->variables(), *expr_ctr};
        ibex_converters_.push_back(ibex_converter.release());
      }
      auto ctc{new ibex::CtcFwdBwd{*num_ctr}};
      ctc_map_.insert(id, ctc);
      return ctc;
    }
    // Adding nullptr so that we don't have to repeat this construction.
    ctc_map_.insert(id, nullptr);
    return nullptr;
  }
}

class TimerHolder {
 public:
  TimerHolder() = default;
  ~TimerHolder() {
    DREAL_LOG_CRITICAL(
        "Per Thread Pruning Time at Contractor: ID = {} : {} sec, # contract "
        "= "
        "{}",
        sched_getcpu(), timer_.seconds(), num_contract_);
  }
  void pause() { timer_.pause(); }
  void resume() {
    timer_.resume();
    ++num_contract_;
  }
  Timer timer_;
  int num_contract_{0};
};

void ContractorIbexFwdbwd::Prune(ContractorStatus* cs) const {
  thread_local TimerHolder timer;
  static ContractorIbexFwdbwdStat stat{DREAL_LOG_INFO_ENABLED};

  ibex::CtcFwdBwd* ctc = GetCtc(cs->box());
  if (ctc) {
    Box::IntervalVector& iv{cs->mutable_box().mutable_interval_vector()};
    Box::IntervalVector old_iv{iv};  // TODO(soonho): FIXME
    old_iv = iv;
    DREAL_LOG_TRACE("ContractorIbexFwdbwd::Prune");
    DREAL_LOG_TRACE("CTC = {}", ctc->ctr);
    DREAL_LOG_TRACE("F = {}", f_);
    timer.resume();
    ctc->contract(iv);  // TODO(soonho): FIXME
    timer.pause();
    stat.increase_pruning();
    bool changed{false};
    // Update output.
    if (iv.is_empty()) {
      changed = true;
      cs->mutable_output().fill(0, cs->box().size() - 1);
    } else {
      for (int i = 0, idx = ctc->output->min(); i < ctc->output->size();
           ++i, idx = ctc->output->next(idx)) {
        if (old_iv[idx] != iv[idx]) {
          cs->mutable_output().add(idx);
          changed = true;
        }
      }
    }
    // Update used constraints.
    if (changed) {
      cs->AddUsedConstraint(f_);
      if (DREAL_LOG_TRACE_ENABLED) {
        ostringstream oss;
        DisplayDiff(oss, cs->box().variables(), old_iv,
                    cs->box().interval_vector());
        DREAL_LOG_TRACE("Changed\n{}", oss.str());
      }
    } else {
      stat.increase_zero_effect_pruning();
      DREAL_LOG_TRACE("NO CHANGE");
    }
  }
}

Box::Interval ContractorIbexFwdbwd::Evaluate(const Box& box) const {
  const thread_local auto thread_id = std::this_thread::get_id();
  const ibex::NumConstraint& num_ctr{ctc_map_.find(thread_id)->ctr};
  return num_ctr.f.eval(box.interval_vector());
}

ostream& ContractorIbexFwdbwd::display(ostream& os) const {
  const thread_local auto thread_id = std::this_thread::get_id();
  if (ctc_map_.contains(thread_id)) {
    const ibex::NumConstraint& num_ctr{ctc_map_.find(thread_id)->ctr};
    return os << "IbexFwdbwd(" << num_ctr << ")";
  } else {
    return os << "IbexFwdbwd()";
  }
}

}  // namespace dreal
