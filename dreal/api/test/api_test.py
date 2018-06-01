# -*- coding: utf-8 -*-
from dreal.symbolic import Variable, logical_and, sin, cos
from dreal.symbolic import logical_imply, forall
from dreal.api import CheckSatisfiability, Minimize
from dreal.util import Box
from dreal.solver import Config
import math
import unittest

x = Variable("x")
y = Variable("y")
z = Variable("z")
p = Variable("p")
x0 = Variable("x0")
x1 = Variable("x1")
x2 = Variable("x2")
y1 = Variable("y1")
y2 = Variable("y2")
u1 = Variable("u1")
u2 = Variable("u2")
u1_ = Variable("u1_")
u2_ = Variable("u2_")

f_sat = logical_and(0 <= x, x <= 10, 0 <= y, y <= 10, 0 <= z, z <= 10,
                    sin(x) + cos(y) == z)

f_unsat = logical_and(3 <= x, x <= 4, 4 <= y, y <= 5, 5 <= z, z <= 6,
                      sin(x) + cos(y) == z)

objective = 2 * x * x + 6 * x + 5
constraint = logical_and(-10 <= x, x <= 10)


class ApiTest(unittest.TestCase):
    # def test_delta_sat(self):
    #     result = CheckSatisfiability(f_sat, 0.001)
    #     self.assertEqual(type(result), Box)

    #     b = Box([x, y, z])
    #     result = CheckSatisfiability(f_sat, 0.001, b)
    #     self.assertEqual(result, True)
    #     self.assertTrue(b[x].diam() < 0.1)
    #     self.assertTrue(b[y].diam() < 0.1)
    #     self.assertTrue(b[z].diam() < 0.1)

    # def test_unsat(self):
    #     result = CheckSatisfiability(f_unsat, 0.001)
    #     self.assertEqual(result, None)

    #     b = Box([x, y, z])
    #     b_copy = Box([x, y, z])
    #     result = CheckSatisfiability(f_unsat, 0.001, b)
    #     self.assertEqual(result, False)
    #     self.assertEqual(b, b_copy)  # Unchanged

    # def test_minimize1(self):
    #     result = Minimize(objective, constraint, 0.00001)
    #     self.assertTrue(result)
    #     self.assertAlmostEqual(result[x].mid(), -1.5, places=2)

    # def test_minimize2(self):
    #     b = Box([x])
    #     result = Minimize(objective, constraint, 0.00001, b)
    #     self.assertTrue(result)
    #     self.assertAlmostEqual(b[x].mid(), -1.5, places=2)

    # def test_minimize3(self):
    #     result = Minimize(x,
    #                       logical_and(0 <= x, x <= 1,
    #                                   0 <= p, p <= 1,
    #                                   p <= x),
    #                       0.00001)
    #     self.assertTrue(result)
    #     self.assertAlmostEqual(result[x].mid(), 0, places=2)
    #     self.assertAlmostEqual(result[p].mid(), 0, places=2)

    # def test_lorentz_cone(self):
    #     config = Config()
    #     config.use_local_optimization = True
    #     config.precision = 0.0001
    #     result = Minimize(x2,
    #                       logical_and(-5 <= x0, x0 <= 5,
    #                                   -5 <= x1, x1 <= 5,
    #                                   0 <= x2, x2 <= 5,
    #                                   1 >= (x0 - 1) ** 2 + (x1 - 1) ** 2,
    #                                   x2 ** 2 >= x0 ** 2 + x1 ** 2),
    #                       config)
    #     self.assertTrue(result)
    #     self.assertAlmostEqual(result[x2].mid(), 0.414212, places=3)

    # def test_minimize_via_forall(self):
    #     # To minimize f(X) s.t. φ(x), this test encodes
    #     # the problem into a first-order logic formula.
    #     #
    #     #    ∃X. φ(X) ∧ [∀Y φ(Y) ⇒ (f(X) ≤ f(Y))]
    #     #
    #     # Here we use f(x) = sin(x)cos(x)
    #     #             φ(X) = (0 ≤ x) ∧ (x ≤ π)
    #     def f(x):
    #         return sin(x) * cos(x)

    #     def phi(x):
    #         return logical_and(0 <= x, x <= math.pi)

    #     problem = logical_and(
    #         phi(x),
    #         forall([y], logical_and(logical_imply(phi(y),
    #                                               f(x) <= f(y)))))
    #     result = CheckSatisfiability(problem, 0.01)
    #     self.assertTrue(result)
    #     self.assertAlmostEqual(result[x].mid(), math.pi * 3 / 4, places=3)

    def test_robust_optimization(self):
        # From https://wiki.mcs.anl.gov/leyffer/images/2/2f/RobustNLP.pdf
        #
        # Minimize x₁ + x₂
        # s.t. u₁x₁ + u₂x₂ - u₁² -u₂² ≤ 0, ∀ u ∈ [-1, 1]²
        #
        # Encoding:
        #
        # ∃x₁,x₂ ∈ [-1, 1]².
        # ∀u₁,u₂ ∈ [-1, 1]². (u₁x₁ + u₂x₂ - u₁² -u₂² ≤ 0) ∧
        # ∀y₁,y₂ ∈ [-1, 1]². (u₁y₁ + u₂y₂ - u₁² -u₂² ≤ 0) ⇒ (x₁ + x₂ ≤ y₁ + y₂)
        def side_condition(v1, v2, p1, p2):
            return p1 * v1 + p2 * v2 - p1 * p1 - p2 * p2 <= 0.0

        def cost_function(v1, v2):
            return v1 + v2

        problem = logical_and(
            -1 <= x1, x1 <= 1,
            -1 <= x2, x2 <= 1,
            forall([u1, u2, u1_, u2_, y1, y2],
                   logical_imply(
                       logical_and(
                           -1 <= u1, u1 <= 1,
                           -1 <= u2, u2 <= 1),
                       logical_and(
                           side_condition(x1, x2, u1, u2),
                           logical_imply(
                               logical_and(
                                   -1 <= y1, y1 <= 1,
                                   -1 <= y2, y2 <= 1,
                                   -1 <= u1_, u1_ <= 1,
                                   -1 <= u2_, u2_ <= 1,
                                   side_condition(y1, y2, u1_, u2_)),
                               cost_function(x1, x2) <= cost_function(y1, y2))
                       ))))


        config = Config()
        config.use_local_optimization = True
        config.precision = 0.001

        print("---------")
        print(problem)
        print("---------")

        result = CheckSatisfiability(problem, config)
        self.assertTrue(result)
        print(result)


if __name__ == '__main__':
    unittest.main()
