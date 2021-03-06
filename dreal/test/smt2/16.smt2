(set-logic QF_NRA)
(set-option :precision 0.01)
(declare-fun x () Real)
(declare-fun y () Real)
(assert (< -4.0 x))
(assert (< x 4.0))
(assert (< -3.0 y))
(assert (< y 3.0))
(assert
        (and
                (= y (arctan2 1 x))
                (= y (* 3 (sin x)))
        )
)
(check-sat)
(exit)
