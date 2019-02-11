#include "dreal/contractor/contractor_shearing.h"

#include <iostream>

#include <gtest/gtest.h>

#include "dreal/contractor/contractor_ibex_fwdbwd.h"
#include "dreal/contractor/contractor_status.h"
#include "dreal/symbolic/symbolic.h"
#include "dreal/util/box.h"

namespace dreal {
namespace {

using std::vector;

class ContractorShearingTest : public ::testing::Test {
 protected:
  const Variable x_{"x", Variable::Type::CONTINUOUS};
  const Variable y_{"y", Variable::Type::CONTINUOUS};
  const Variable z_{"z", Variable::Type::CONTINUOUS};
  const vector<Variable> vars_{{x_, y_}};
  Box box_{vars_};
};

TEST_F(ContractorShearingTest, Shearing1) {
  // const Formula f{x_ * x_ + x_ == pow(y_, 3) - 2 * y_};
  const Formula f{sin(x_) == cos(y_)};

  box_[x_] = Box::Interval(0.0, 3.14 / 2);
  box_[y_] = Box::Interval(0.2, 0.3);
  ContractorStatus cs{box_};
  ContractorShearing ctc{
      f, box_, Config{}, Contractor::ShearingMethod::Taylor1, 1000, 0.05};
  std::cerr << "BEFORE PRUNING (TAYLOR):\n" << cs.box() << "\n";
  ctc.Prune(&cs);
  std::cerr << "AFTER PRUNING (TAYLOR):\n" << cs.box() << "\n";
}

TEST_F(ContractorShearingTest, IbexFwd) {
  const Formula f{sin(x_) == cos(y_)};
  box_[x_] = Box::Interval(0.0, 3.14 / 2);
  box_[y_] = Box::Interval(0.2, 0.3);
  ContractorStatus cs{box_};
  ContractorIbexFwdbwd ctc{f, box_, Config{}};
  std::cerr << "BEFORE PRUNING (IBEXFWDBWD):\n" << cs.box() << "\n";
  ctc.Prune(&cs);
  std::cerr << "AFTER PRUNING (IBEXFWDBWD):\n" << cs.box() << "\n";
}

}  // namespace
}  // namespace dreal
