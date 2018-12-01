#include "dreal/symbolic/symbolic.h"

#include <iostream>
#include <type_traits>

#include <gtest/gtest.h>

#include "dreal/symbolic/symbolic_test_util.h"
#include "dreal/util/eigen.h"

using std::cout;
using std::endl;
using std::to_string;
using std::vector;

namespace dreal {
namespace {

class SymbolicTest : public ::testing::Test {
 protected:
  const Variable x_{"x", Variable::Type::CONTINUOUS};
  const Variable y_{"y", Variable::Type::CONTINUOUS};
  const Variable z_{"z", Variable::Type::CONTINUOUS};

  const Variable b1_{"B1", Variable::Type::BOOLEAN};
  const Variable b2_{"B2", Variable::Type::BOOLEAN};
  const Variable b3_{"B3", Variable::Type::BOOLEAN};
};

TEST_F(SymbolicTest, Imply) {
  // b₁ ⇒ b₂
  const Formula f{imply(Formula{b1_}, Formula{b2_})};

  // T ⇒ T  =  T
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::True()).Substitute(b2_, Formula::True()),
      Formula::True());
  // T ⇒ F  =  F
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::True()).Substitute(b2_, Formula::False()),
      Formula::False());
  // F ⇒ T  =  T
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::False()).Substitute(b2_, Formula::True()),
      Formula::True());
  // F ⇒ F  =  T
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::False()).Substitute(b2_, Formula::False()),
      Formula::True());
}

TEST_F(SymbolicTest, Iff) {
  // b₁ ⇔ b₂
  const Formula f{iff(Formula{b1_}, Formula{b2_})};

  // T ⇔ T  =  T
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::True()).Substitute(b2_, Formula::True()),
      Formula::True());
  // T ⇔ F  =  F
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::True()).Substitute(b2_, Formula::False()),
      Formula::False());
  // F ⇔ T  =  F
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::False()).Substitute(b2_, Formula::True()),
      Formula::False());
  // F ⇔ F  =  T
  EXPECT_PRED2(
      FormulaEqual,
      f.Substitute(b1_, Formula::False()).Substitute(b2_, Formula::False()),
      Formula::True());
}

TEST_F(SymbolicTest, Equality) {
  {
    // Boolean Variable == Boolean Variable.
    const Formula f{b1_ == b2_};
    EXPECT_PRED2(FormulaEqual, f, iff(b1_, b2_));
  }

  {
    // Scalar Variable == Scalar Variable.
    const Formula f{x_ == y_};
    EXPECT_TRUE(is_relational(f));
  }

  {
    // Expression == Scalar Variable.
    const Formula f{(x_ + 1) == y_};
    EXPECT_TRUE(is_relational(f));
  }

  // Scalar Variable == Expression.
  {
    const Formula f{y_ == (x_ + 1)};
    EXPECT_TRUE(is_relational(f));
  }

  // Expression == Expression.
  {
    const Formula f{y_ == x_};
    EXPECT_TRUE(is_relational(f));
  }

  {
    // Boolean Variable == Formula.
    const Formula f{b1_ == (x_ > y_)};
    EXPECT_PRED2(FormulaEqual, f, iff(b1_, x_ > y_));
  }

  {
    // Formula == Boolean Variable.
    const Formula f{(x_ > y_) == b1_};
    EXPECT_PRED2(FormulaEqual, f, iff(b1_, x_ > y_));
  }

  {
    // Formula == Formula.
    const Formula f{(y_ > z_) == (x_ > y_)};
    EXPECT_PRED2(FormulaEqual, f, iff(y_ > z_, x_ > y_));
  }

  {
    Formula f;
    // Boolean Variable == Scalar Variable: => EXCEPTION.
    EXPECT_THROW(f = (b1_ == y_), std::runtime_error);

    // Scalar Variable == Boolean Variable: => EXCEPTION.
    EXPECT_THROW(f = (y_ == b1_), std::runtime_error);

    // Boolean Variable == Expression: => EXCEPTION.
    EXPECT_THROW(f = (b1_ == (y_ + 3)), std::runtime_error);

    // Expression == Boolean Variable: => EXCEPTION.
    EXPECT_THROW(f = ((x_ + 3) == b1_), std::runtime_error);

    // Scalar Variable == Formula: => EXCEPTION.
    EXPECT_THROW(f = (x_ == (x_ > 3)), std::runtime_error);

    // Formula == Scalar Variable: => EXCEPTION.
    EXPECT_THROW(f = ((x_ > 3) == x_), std::runtime_error);

    // Expression == Formula: => Compile Error.
    // EXPECT_THROW(f = ((x_ + 3) == (x_ > 3)), std::runtime_error);

    // Formula == Expression: => Compile Error.
    // EXPECT_THROW(f = ((x_ > 3) == (x_ + 3)), std::runtime_error);
  }
}

TEST_F(SymbolicTest, Inequality) {
  {
    // Boolean Variable != Boolean Variable.
    const Formula f{b1_ != b2_};
    EXPECT_PRED2(FormulaEqual, f, !iff(b1_, b2_));
  }

  {
    // Scalar Variable != Scalar Variable.
    const Formula f{x_ != y_};
    EXPECT_TRUE(is_relational(f));
  }

  {
    // Expression != Scalar Variable.
    const Formula f{(x_ + 1) != y_};
    EXPECT_TRUE(is_relational(f));
  }

  // Scalar Variable != Expression.
  {
    const Formula f{y_ != (x_ + 1)};
    EXPECT_TRUE(is_relational(f));
  }

  // Expression != Expression.
  {
    const Formula f{y_ != x_};
    EXPECT_TRUE(is_relational(f));
  }

  {
    // Boolean Variable != Formula.
    const Formula f{b1_ != (x_ > y_)};
    EXPECT_PRED2(FormulaEqual, f, !iff(b1_, x_ > y_));
  }

  {
    // Formula != Boolean Variable.
    const Formula f{(x_ > y_) != b1_};
    EXPECT_PRED2(FormulaEqual, f, !iff(b1_, x_ > y_));
  }

  {
    // Formula != Formula.
    const Formula f{(y_ > z_) != (x_ > y_)};
    EXPECT_PRED2(FormulaEqual, f, !iff(y_ > z_, x_ > y_));
  }

  {
    Formula f;
    // Boolean Variable != Scalar Variable: => EXCEPTION.
    EXPECT_THROW(f = (b1_ != y_), std::runtime_error);

    // Scalar Variable != Boolean Variable: => EXCEPTION.
    EXPECT_THROW(f = (y_ != b1_), std::runtime_error);

    // Boolean Variable != Expression: => EXCEPTION.
    EXPECT_THROW(f = (b1_ != (y_ + 3)), std::runtime_error);

    // Expression != Boolean Variable: => EXCEPTION.
    EXPECT_THROW(f = ((x_ + 3) != b1_), std::runtime_error);

    // Scalar Variable != Formula: => EXCEPTION.
    EXPECT_THROW(f = (x_ != (x_ > 3)), std::runtime_error);

    // Formula != Scalar Variable: => EXCEPTION.
    EXPECT_THROW(f = ((x_ > 3) != x_), std::runtime_error);

    // Expression != Formula: => Compile Error.
    // EXPECT_THROW(f = ((x_ + 3) != (x_ > 3)), std::runtime_error);

    // Formula != Expression: => Compile Error.
    // EXPECT_THROW(f = ((x_ > 3) != (x_ + 3)), std::runtime_error);
  }
}

TEST_F(SymbolicTest, CreateVectorContinuous) {
  const vector<Variable> v{CreateVector("x", 5)};
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(v[i].get_name(), "x" + to_string(i));
    EXPECT_EQ(v[i].get_type(), Variable::Type::CONTINUOUS);
  }
}

TEST_F(SymbolicTest, CreateVectorInteger) {
  const vector<Variable> v{CreateVector("y", 10, Variable::Type::INTEGER)};
  for (int i = 0; i < 10; ++i) {
    EXPECT_EQ(v[i].get_name(), "y" + to_string(i));
    EXPECT_EQ(v[i].get_type(), Variable::Type::INTEGER);
  }
}

TEST_F(SymbolicTest, Sum) {
  const Expression e1{x_ + 1.0};
  const Expression e2{y_ + 2.0};
  const Expression e3{x_ * y_ * z_};
  EXPECT_PRED2(ExprEqual, Sum({}), Expression::Zero());
  EXPECT_PRED2(ExprEqual, Sum({e1}), e1);
  EXPECT_PRED2(ExprEqual, (Sum({e1, e2, e3})), e1 + e2 + e3);
}

TEST_F(SymbolicTest, Prod) {
  const Expression e1{x_ + 1.0};
  const Expression e2{y_ + 2.0};
  const Expression e3{x_ * y_ * z_};
  EXPECT_PRED2(ExprEqual, Prod({}), Expression::One());
  EXPECT_PRED2(ExprEqual, Prod({e1}), e1);
  EXPECT_PRED2(ExprEqual, (Prod({e1, e2, e3})), e1 * e2 * e3);
}

TEST_F(SymbolicTest, DestructiveUpdateAddition1) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e += Variable("var_" + std::to_string(i));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateAddition2) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e = std::move(e) + Variable("var_" + std::to_string(i));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateAddition3) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e = Variable("var_" + std::to_string(i)) + std::move(e);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateSubtraction1) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e -= Variable("var_" + std::to_string(i));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateSubtraction2) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e = std::move(e) - Variable("var_" + std::to_string(i));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateSubtraction3) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e = Variable("var_" + std::to_string(i)) + (-std::move(e));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateSubtraction4) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e = Variable("var_" + std::to_string(i)) - std::move(e);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateUnaryMinus1) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e += Variable("var_" + std::to_string(i));
  }
  for (int i = 0; i < N; ++i) {
    e = -std::move(e);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateUnaryMinus2) {
  constexpr int N{1000};
  Expression e;
  for (int i = 0; i < N; ++i) {
    e += Variable("var_" + std::to_string(i));
  }
  for (int i = 0; i < N; ++i) {
    e *= -1;
  }
}

TEST_F(SymbolicTest, DestructiveUpdateMultiplication1) {
  constexpr int N{1000};
  Expression e{1.0};
  for (int i = 0; i < N; ++i) {
    e *= Variable("var_" + std::to_string(i));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateMultiplication2) {
  constexpr int N{1000};
  Expression e{1.0};
  for (int i = 0; i < N; ++i) {
    e = std::move(e) * Variable("var_" + std::to_string(i));
  }
}

TEST_F(SymbolicTest, DestructiveUpdateMultiplication3) {
  constexpr int N{1000};
  Expression e{1.0};
  for (int i = 0; i < N; ++i) {
    e = Variable("var_" + std::to_string(i)) * std::move(e);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateAnd1) {
  constexpr int N{1000};
  Formula f{Formula::True()};
  for (int i = 0; i < N; ++i) {
    f = std::move(f) && (Variable("var_" + std::to_string(i)) == 0.0);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateAnd2) {
  constexpr int N{1000};
  Formula f{Formula::True()};
  for (int i = 0; i < N; ++i) {
    f = (Variable("var_" + std::to_string(i)) == 0.0) && std::move(f);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateOr1) {
  constexpr int N{1000};
  Formula f{Formula::False()};
  for (int i = 0; i < N; ++i) {
    f = std::move(f) || (Variable("var_" + std::to_string(i)) == 0.0);
  }
}

TEST_F(SymbolicTest, DestructiveUpdateOr2) {
  constexpr int N{1000};
  Formula f{Formula::False()};
  for (int i = 0; i < N; ++i) {
    f = (Variable("var_" + std::to_string(i)) == 0.0) || std::move(f);
  }
}

GTEST_TEST(Symbolic, is_nothrow_move_constructible) {
  static_assert(std::is_nothrow_move_constructible<Variable>::value,
                "Variable should be nothrow_move_constructible.");
  static_assert(std::is_nothrow_move_constructible<Expression>::value,
                "Expression should be nothrow_move_constructible.");
  static_assert(std::is_nothrow_move_constructible<Formula>::value,
                "Formula should be nothrow_move_constructible.");
}

class SymbolicExpressionJacobianTest : public ::testing::Test {
 protected:
  const Variable x_{"x"};
  const Variable y_{"y"};
  const Variable z_{"z"};
};

TEST_F(SymbolicExpressionJacobianTest, Test1) {
  // Jacobian(2*x + 3*y + 4*z, [x, y, z])
  //  = [2, 3, 4]
  VectorX<Expression> f(1);
  f << 2 * x_ + 3 * y_ + 4 * z_;

  MatrixX<Expression> expected(1, 3);
  expected << 2, 3, 4;

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, y_, z_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector3<Variable>{x_, y_, z_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test2) {
  // Jacobian([x * y * z, y^2, x + z], {x, y, z})
  //  = |(y * z)   (x * z)   (x * y)|
  //    |      0   (2 * y)         0|
  //    |      1         0         1|
  VectorX<Expression> f(3);
  f << x_ * y_ * z_, pow(y_, 2), x_ + z_;

  MatrixX<Expression> expected(3, 3);
  // clang-format off
  expected << y_ * z_, x_ * z_, x_ * y_,
                    0,  2 * y_,       0,
                    1,       0,       1;
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, y_, z_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector3<Variable>{x_, y_, z_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test3) {
  // Jacobian([x^2*y, x*sin(y)], {x})
  // =  | 2*x*y  |
  //    | sin(y) |
  VectorX<Expression> f(2);
  f << pow(x_, 2) * y_, x_ * sin(y_);

  MatrixX<Expression> expected(2, 1);
  // clang-format off
  expected << 2 * x_ * y_,
                  sin(y_);
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector1<Variable>{x_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test4) {
  // Jacobian([x * cos(y), x * sin(y), x^2], {x, y})
  //  = |cos(y)   -x * sin(y)|
  //    |sin(y)    x * cos(y)|
  //    | 2 * x             0|
  VectorX<Expression> f(3);
  f << x_ * cos(y_), x_ * sin(y_), pow(x_, 2);

  MatrixX<Expression> expected(3, 2);
  // clang-format off
  expected << cos(y_), -x_ * sin(y_),
              sin(y_),  x_ * cos(y_),
               2 * x_,             0;
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, y_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector2<Variable>{x_, y_}), expected);
}

TEST_F(SymbolicExpressionJacobianTest, Test5) {
  // Jacobian([x * y + sin(x)], {x, z})
  //  = |y + cos(x)  0|
  VectorX<Expression> f(1);
  f << x_ * y_ + sin(x_);

  MatrixX<Expression> expected(1, 2);
  // clang-format off
  expected << y_ + cos(x_),
                         0;
  // clang-format on

  // std::vector of variables
  EXPECT_EQ(Jacobian(f, {x_, z_}), expected);
  // Eigen::Vector of variables
  EXPECT_EQ(Jacobian(f, Vector2<Variable>{x_, z_}), expected);
}

}  // namespace
}  // namespace dreal
