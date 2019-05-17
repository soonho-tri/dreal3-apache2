#include "dreal/contractor/contractor_ibex_fwdbwd.h"

#include <sstream>
#include <utility>

#include "dreal/util/assert.h"
#include "dreal/util/logging.h"
#include "dreal/util/math.h"
#include "dreal/util/stat.h"
#include "dreal/util/timer.h"

using std::cout;
using std::lock_guard;
using std::make_unique;
using std::mutex;
using std::ostream;
using std::ostringstream;
using std::thread;

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

  void increase_zero_effect_pruning() { ++num_zero_effect_pruning_; }

  void increase_pruning() { ++num_pruning_; }

 private:
  int num_zero_effect_pruning_{0};
  int num_pruning_{0};
};

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

}  // namespace

//---------------------------------------
// Implementation of ContractorIbexFwdbwd
//---------------------------------------
ContractorIbexFwdbwd::ContractorIbexFwdbwd(Formula f, const Box& box,
                                           const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_FWDBWD,
                     ibex::BitSet::empty(box.size()), config},
      f_{std::move(f)},
      ibex_converter_{box} {
  // Build num_ctr and ctc_.
  expr_ctr_.reset(ibex_converter_.Convert(f_));
  if (expr_ctr_) {
    auto const num_ctr =
        new ibex::NumConstraint{ibex_converter_.variables(), *expr_ctr_};
    ctc_ = make_unique<ibex::CtcFwdBwd>(*num_ctr);
    // Build input.
    ibex::BitSet& input{mutable_input()};
    for (const Variable& var : f_.GetFreeVariables()) {
      input.add(box.index(var));
    }
  }
}

ContractorIbexFwdbwd::~ContractorIbexFwdbwd() {
  if (ctc_) {
    const ibex::NumConstraint* const num_ctr{&ctc_->ctr};
    delete num_ctr;
  }
}

void ContractorIbexFwdbwd::Prune(ContractorStatus* cs) const {
  // thread_local TimerHolder timer;
  thread_local ContractorIbexFwdbwdStat stat{DREAL_LOG_INFO_ENABLED};

  if (ctc_) {
    Box::IntervalVector& iv{cs->mutable_box().mutable_interval_vector()};
    Box::IntervalVector old_iv{iv};  // TODO(soonho): FIXME
    old_iv = iv;
    DREAL_LOG_TRACE("ContractorIbexFwdbwd::Prune");
    DREAL_LOG_TRACE("CTC = {}", ctc_->ctr);
    DREAL_LOG_TRACE("F = {}", f_);
    // timer.resume();
    ctc_->contract(iv);  // TODO(soonho): FIXME
    // timer.pause();
    // stat.increase_pruning();
    bool changed{false};
    // Update output.
    if (iv.is_empty()) {
      changed = true;
      cs->mutable_output().fill(0, cs->box().size() - 1);
    } else {
      for (int i{0}, idx = ctc_->output->min(); i < ctc_->output->size();
           ++i, idx = ctc_->output->next(idx)) {
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
  return ctc_->ctr.f.eval(box.interval_vector());
}

ostream& ContractorIbexFwdbwd::display(ostream& os) const {
  if (ctc_) {
    const ibex::NumConstraint& num_ctr{ctc_->ctr};
    return os << "IbexFwdbwd(" << num_ctr << ")";
  } else {
    return os << "IbexFwdbwd()";
  }
}

//---------------------------------------
// Implementation of ContractorIbexFwdbwdMt
//---------------------------------------
ContractorIbexFwdbwdMt::ContractorIbexFwdbwdMt(Formula f, const Box& box,
                                               const Config& config)
    : ContractorCell{Contractor::Kind::IBEX_FWDBWD,
                     ibex::BitSet::empty(box.size()), config},
      f_{std::move(f)},
      config_{config},
      ctc_map_{static_cast<
          cuckoohash_map<std::thread::id, ContractorIbexFwdbwd*>::size_type>(
          config_.number_of_jobs())} {
  DREAL_LOG_DEBUG("ContractorIbexFwdbwdMt::ContractorIbexFwdbwdMt");
  ctcs_.reserve(config.number_of_jobs());
  GetCtc(box);
}

ContractorIbexFwdbwdMt::~ContractorIbexFwdbwdMt() {
  for (const ContractorIbexFwdbwd* ctc : ctcs_) {
    delete ctc;
  }
}

ContractorIbexFwdbwd* ContractorIbexFwdbwdMt::GetCtc(const Box& box) const {
  thread_local const std::thread::id id{std::this_thread::get_id()};
  if (ctc_map_.contains(id)) {
    return ctc_map_.find(id);
  }
  ContractorIbexFwdbwd* const ctc{new ContractorIbexFwdbwd{f_, box, config_}};
  ctc_map_.insert(id, ctc);
  {
    std::lock_guard<std::mutex> guard{m_};
    ctcs_.push_back(ctc);
    return ctc;
  }
}

ContractorIbexFwdbwd* ContractorIbexFwdbwdMt::GetCtc() const {
  thread_local const std::thread::id id{std::this_thread::get_id()};
  return ctc_map_.find(id);
}

void ContractorIbexFwdbwdMt::Prune(ContractorStatus* cs) const {
  ContractorIbexFwdbwd* const ctc{GetCtc(cs->box())};
  if (ctc) {
    return ctc->Prune(cs);
  }
}

Box::Interval ContractorIbexFwdbwdMt::Evaluate(const Box& box) const {
  ContractorIbexFwdbwd* const ctc{GetCtc(box)};
  return ctc->Evaluate(box);
}

ostream& ContractorIbexFwdbwdMt::display(ostream& os) const {
  ContractorIbexFwdbwd* const ctc{GetCtc()};
  if (ctc) {
    return ctc->display(os);
  } else {
    return os << "IbexFwdbwd()";
  }
}

}  // namespace dreal
