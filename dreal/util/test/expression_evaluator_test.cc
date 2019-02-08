#include "dreal/util/expression_evaluator.h"

#include <iostream>
#include <sstream>

#include <gtest/gtest.h>

namespace dreal {
namespace {

using std::cerr;
using std::endl;
using std::ostringstream;

class ExpressionEvaluatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    box_.Add(x_);
    box_.Add(y_);
    box_.Add(z_);
  }

  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
  Box box_;
};

TEST_F(ExpressionEvaluatorTest, Arithmetic1) {
  const Expression e{x_ + y_ + z_};
  const ExpressionEvaluator evaluator{e};

  box_[x_] = Box::Interval{1, 2};
  box_[y_] = Box::Interval{2, 3};
  box_[z_] = Box::Interval{3, 4};

  EXPECT_EQ(evaluator(box_), Box::Interval(1 + 2 + 3, 2 + 3 + 4));
  ostringstream oss;
  oss << evaluator;
  EXPECT_EQ(oss.str(), "ExpressionEvaluator((x + y + z))");
}

using std::cerr;
using std::endl;

TEST_F(ExpressionEvaluatorTest, TaylorEval) {
  const Variable x1{"x1"};
  const Variable x2{"x2"};

  // f = 3x₁² + x₂² + x₁x₂.
  // const Expression f{3 * x1 * x1 + x2 * x2 + x1 * x2};
  // const Expression f{x1 - atan(x1)};
  const Expression f{x1 * x1 + x1};

  // Set up a box [x] = [-1, 3] x [-1, 5].
  Box x;
  x.Add(x1);
  x.Add(x2);
  x[x1] = Box::Interval(0, 1);
  x[x2] = Box::Interval(0, 1);

  const Box::Interval eval_with_natural_extension{Eval(f, x)};
  const Box::Interval eval_with_taylor1_extension{Taylor1Eval(f, x)};
  const Box::Interval eval_with_taylor2_extension{Taylor2Eval(f, x)};

  // Output: Eval(f, [x])   = [-5, 67]
  cerr << "Eval(f, [x])   = " << eval_with_natural_extension << endl;
  // Output: Taylor(f, [x]) = [-76, 94]
  cerr << "Taylor₁(f, [x]) = " << eval_with_taylor1_extension << endl;

  cerr << "Taylor₂(f, [x]) = " << eval_with_taylor2_extension << endl;
}

TEST_F(ExpressionEvaluatorTest, TaylorEval2) {
  const Variable x1{"x1"};
  const Variable x2{"x2"};

  // f = 3x₁² + x₂² + x₁x₂.
  // const Expression f{3 * x1 * x1 + x2 * x2 + x1 * x2};
  // const Expression f{x1 - atan(x1)};
  const Expression f1{x1 * x1 + x1 + pow(x2, 2)};
  const Expression f2{x1 * x1 + x1};
  const Expression f3{pow(x2, 2)};

  // Set up a box [x] = [-1, 3] x [-1, 5].
  Box x;
  x.Add(x1);
  x.Add(x2);
  x[x1] = Box::Interval(0, 1);
  x[x2] = Box::Interval(0, 1);

  {
    const Box::Interval eval_with_natural_extension{Eval(f1, x)};
    const Box::Interval eval_with_taylor1_extension{Taylor1Eval(f1, x)};
    const Box::Interval eval_with_taylor2_extension{Taylor2Eval(f1, x)};

    cerr << "Eval(f1, [x])   = " << eval_with_natural_extension << endl;

    cerr << "Taylor₁(f1, [x]) = " << eval_with_taylor1_extension << endl;

    cerr << "Taylor₂(f1, [x]) = " << eval_with_taylor2_extension << endl;
  }

  {
    const Box::Interval eval_with_natural_extension{Eval(f2, x) + Eval(f3, x)};
    const Box::Interval eval_with_taylor1_extension{Taylor1Eval(f2, x) +
                                                    Eval(f3, x)};
    const Box::Interval eval_with_taylor2_extension{Taylor2Eval(f2, x) +
                                                    Eval(f3, x)};

    cerr << "Eval(f2, [x]) + Eval(f3, [x])    = " << eval_with_natural_extension
         << endl;

    cerr << "Taylor₁(f2, [x]) + Eval(f3, [x]) = " << eval_with_taylor1_extension
         << endl;

    cerr << "Taylor₂(f2, [x]) + Eval(f3, [x]) = " << eval_with_taylor2_extension
         << endl;
  }
}

// TODO(soonho): Add more tests.

}  // namespace
}  // namespace dreal
