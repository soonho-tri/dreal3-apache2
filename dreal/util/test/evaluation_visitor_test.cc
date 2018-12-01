#include "dreal/util/evaluation_visitor.h"

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>

#include <gtest/gtest.h>

#include "./ibex.h"

namespace dreal {

using Interval = ibex::Interval;

class EvaluationVisitorTest : public ::testing::Test {
 protected:
  const Variable x_{"x", Variable::Type::CONTINUOUS};
  const Variable y_{"y", Variable::Type::CONTINUOUS};
  const Variable z_{"z", Variable::Type::CONTINUOUS};
};

TEST_F(EvaluationVisitorTest, Double) {
  using T = double;
  VectorX<T> v(3);
  v << 1.0, 2.0, 3.0;
  EvaluationVisitor visitor{{x_, y_, z_}};
  const T result{visitor.Evaluate<double>(x_ + 2 * y_ + 3 * z_, v)};

  EXPECT_EQ(result, 1.0 + 2 * 2.0 + 3 * 3.0 /* 14.0 */);
}

TEST_F(EvaluationVisitorTest, DoubleVector) {
  using T = double;
  VectorX<T> v(3);
  v << 1.0, 2.0, 3.0;
  VectorX<Expression> expressions(3);
  expressions << sin(x_) + cos(y_), x_ * y_ * z_, 3 * x_ + 4 * y_ * 5 * z_;

  EvaluationVisitor visitor{{x_, y_, z_}};
  const VectorX<T> result{visitor.Evaluate<T>(expressions, v)};

  EXPECT_EQ(result(0), sin(1.0) + cos(2.0));
}

TEST_F(EvaluationVisitorTest, AutoDiffDouble) {
  using T = Eigen::AutoDiffScalar<Eigen::Vector3d>;
  const Expression e{1 + 2 * x_ + 3 * y_ + 4 * z_};
  Vector3<T> v(3);
  v << 1.0, 2.0, 3.0;
  Vector3<double> v_d(3);
  v_d << 1.0, 2.0, 3.0;

  v(0).derivatives() = Eigen::Vector3d::Unit(0);  // [ 1 0 0 ]
  v(1).derivatives() = Eigen::Vector3d::Unit(1);  // [ 0 1 0 ]
  v(2).derivatives() = Eigen::Vector3d::Unit(2);  // [ 0 0 1 ]

  EvaluationVisitor visitor{{x_, y_, z_}};
  const T result{visitor.Evaluate<T>(e, v)};

  EXPECT_EQ(result, 1 + 2 * 1.0 + 3 * 2.0 + 4 * 3.0);

  EXPECT_EQ(result.derivatives()(0),
            visitor.Evaluate<double>(e.Differentiate(x_), v_d));
  EXPECT_EQ(result.derivatives()(1), 3);
  EXPECT_EQ(result.derivatives()(2), 4);
}

TEST_F(EvaluationVisitorTest, AutoDiffDoubleVector) {
  using T = Eigen::AutoDiffScalar<Eigen::Vector3d>;
  VectorX<double> v_d(3);
  v_d << 1.0, 2.0, 3.0;

  VectorX<T> v(3);
  v << 1.0, 2.0, 3.0;
  v(0).derivatives() = Eigen::Vector3d::Unit(0);  // [ 1 0 0 ]
  v(1).derivatives() = Eigen::Vector3d::Unit(1);  // [ 0 1 0 ]
  v(2).derivatives() = Eigen::Vector3d::Unit(2);  // [ 0 0 1 ]

  VectorX<Expression> expressions(3);
  expressions << sin(x_) + cos(y_), x_ * y_ * z_, 3 * x_ + 4 * y_ * 5 * z_;

  EvaluationVisitor visitor{{x_, y_, z_}};
  const VectorX<T> result{visitor.Evaluate<T>(expressions, v)};
  EXPECT_EQ(result(0).value(), sin(1.0) + cos(2.0));
  EXPECT_EQ(result(0).derivatives()(0),
            visitor.Evaluate<double>(expressions(0).Differentiate(x_), v_d));
  EXPECT_EQ(result(0).derivatives()(1),
            visitor.Evaluate<double>(expressions(0).Differentiate(y_), v_d));
  EXPECT_EQ(result(0).derivatives()(2),
            visitor.Evaluate<double>(expressions(0).Differentiate(z_), v_d));

  EXPECT_EQ(result(1).value(), 1.0 * 2.0 * 3.0);
  EXPECT_EQ(result(1).derivatives()(0),
            visitor.Evaluate<double>(expressions(1).Differentiate(x_), v_d));
  EXPECT_EQ(result(1).derivatives()(1),
            visitor.Evaluate<double>(expressions(1).Differentiate(y_), v_d));
  EXPECT_EQ(result(1).derivatives()(2),
            visitor.Evaluate<double>(expressions(1).Differentiate(z_), v_d));

  EXPECT_EQ(result(2).value(), 3 * 1.0 + 4 * 2.0 * 5 * 3.0);
  EXPECT_EQ(result(2).derivatives()(0),
            visitor.Evaluate<double>(expressions(2).Differentiate(x_), v_d));
  EXPECT_EQ(result(2).derivatives()(1),
            visitor.Evaluate<double>(expressions(2).Differentiate(y_), v_d));
  EXPECT_EQ(result(2).derivatives()(2),
            visitor.Evaluate<double>(expressions(2).Differentiate(z_), v_d));
}

TEST_F(EvaluationVisitorTest, Interval1) {
  using T = Interval;
  VectorX<T> v(3);
  v << Interval{1.0, 2.0}, Interval{2.0, 3.0}, Interval{3.0, 4.0};
  EvaluationVisitor visitor{{x_, y_, z_}};
  const Expression e{
      pow(x_, y_) + sin(x_) + 2 * cos(y_) + 3 * atan(z_) +
      real_constant(
          1.0, std::nextafter(1.0, std::numeric_limits<double>::infinity()),
          true)};
  const T result{visitor.Evaluate<T>(e, v)};
  std::cerr << result << std::endl;
}

TEST_F(EvaluationVisitorTest, Interval2) {
  using T = Interval;
  VectorX<T> v(3);
  v << Interval{1.0, 2.0}, Interval{2.0, 3.0}, Interval{3.0, 4.0};
  EvaluationVisitor visitor{{x_, y_, z_}};
  const Expression e{1.0};
  const T result{visitor.Evaluate<T>(e, v)};
  EXPECT_EQ(result.diam(), 0.0);
}

TEST_F(EvaluationVisitorTest, Interval3) {
  using T = Interval;
  VectorX<T> v(3);
  v << Interval{1.0, 2.0}, Interval{2.0, 3.0}, Interval{3.0, 4.0};
  EvaluationVisitor visitor{{x_, y_, z_}};
  const Expression e{real_constant(
      1.0, std::nextafter(1.0, std::numeric_limits<double>::infinity()), true)};
  const T result{visitor.Evaluate<T>(e, v)};
  EXPECT_GT(result.diam(), 0.0);
}

}  // namespace dreal
