(set-logic QF_NRA)
(declare-fun v4 () Real)
(declare-fun th4 () Real)
(declare-fun zp_4_4 () Real)
(declare-fun zq_4_4 () Real)
(declare-fun zp_4_15 () Real)
(declare-fun zq_4_15 () Real)
(declare-fun v15 () Real)
(declare-fun th15 () Real)
(declare-fun zp_15_4 () Real)
(declare-fun zq_15_4 () Real)
(declare-fun zp_15_15 () Real)
(declare-fun zq_15_15 () Real)
(declare-fun v24 () Real)
(declare-fun th24 () Real)
(declare-fun zp_24_4 () Real)
(declare-fun zq_24_4 () Real)
(declare-fun zp_24_24 () Real)
(declare-fun zq_24_24 () Real)
(assert (<= v4 2))
(assert (>= v4 0))
(assert (<= th4 4))
(assert (>= th4 -4))
(assert (<= zp_4_4 3))
(assert (>= zp_4_4 -3))
(assert (<= zq_4_4 3))
(assert (>= zq_4_4 -3))
(assert (<= zp_4_15 3))
(assert (>= zp_4_15 -3))
(assert (<= zq_4_15 3))
(assert (>= zq_4_15 -3))
(assert (<= v15 2))
(assert (>= v15 0))
(assert (<= th15 4))
(assert (>= th15 -4))
(assert (<= zp_15_4 3))
(assert (>= zp_15_4 -3))
(assert (<= zq_15_4 3))
(assert (>= zq_15_4 -3))
(assert (<= zp_15_15 3))
(assert (>= zp_15_15 -3))
(assert (<= zq_15_15 3))
(assert (>= zq_15_15 -3))
(assert (<= v24 2))
(assert (>= v24 0))
(assert (<= th24 4))
(assert (>= th24 -4))
(assert (<= zp_24_4 3))
(assert (>= zp_24_4 -3))
(assert (<= zq_24_4 3))
(assert (>= zq_24_4 -3))
(assert (<= zp_24_24 3))
(assert (>= zp_24_24 -3))
(assert (<= zq_24_24 3))
(assert (>= zq_24_24 -3))
(assert (and (and (and (and (and (and (and (and (and (and (and (and true (and (= zp_24_24 (* v24 (+ 0 (* v4 (- (- 0 (* 0.052763 (cos (- th24 th4)))) (* 0.084564 (sin (- th24 th4)))))))) (= zq_24_24 (* v24 (+ 0 (* v4 (- (* 0.084564 (cos (- th24 th4))) (* 0.052763 (sin (- th24 th4)))))))))) (and (= zp_24_24 1) (= zq_24_24 1))) (and (= zp_24_4 (- (- (* (^ v24 2) 0.052763) (* v24 (* v4 (* 0.052763 (cos (- th24 th4)))))) (* v24 (* v4 (* 0.084564 (sin (- th24 th4))))))) (= zq_24_4 (- (- (- 0 (* (^ v24 2) 0.052763)) (* v24 (* v4 (* 0.084564 (cos (- th24 th4)))))) (* v24 (* v4 (* 0.052763 (sin (- th24 th4))))))))) (and (= zp_4_4 (* v4 (+ (+ 0 (* v15 (- (- 0 (* 0.033082 (cos (- th4 th15)))) (* 0.146003 (sin (- th4 th15)))))) (* v24 (- (- 0 (* 0.052763 (cos (- th4 th24)))) (* 0.084564 (sin (- th4 th24)))))))) (= zq_4_4 (* v4 (+ (+ 0 (* v15 (- (* 0.146003 (cos (- th4 th15))) (* 0.033082 (sin (- th4 th15)))))) (* v24 (- (* 0.084564 (cos (- th4 th24))) (* 0.052763 (sin (- th4 th24)))))))))) (and (= zp_4_4 1) (= zq_4_4 1))) (and (= zp_4_15 (- (- (* (^ v4 2) 0.033082) (* v4 (* v15 (* 0.033082 (cos (- th4 th15)))))) (* v4 (* v15 (* 0.146003 (sin (- th4 th15))))))) (= zq_4_15 (- (- (- 0 (* (^ v4 2) 0.033082)) (* v4 (* v15 (* 0.146003 (cos (- th4 th15)))))) (* v4 (* v15 (* 0.033082 (sin (- th4 th15))))))))) (and (= zp_15_15 (* v15 (+ 0 (* v4 (- (- 0 (* 0.033082 (cos (- th15 th4)))) (* 0.146003 (sin (- th15 th4)))))))) (= zq_15_15 (* v15 (+ 0 (* v4 (- (* 0.146003 (cos (- th15 th4))) (* 0.033082 (sin (- th15 th4)))))))))) (and (= zp_15_15 1) (= zq_15_15 1))) (and (= zp_15_4 (- (- (* (^ v15 2) 0.033082) (* v15 (* v4 (* 0.033082 (cos (- th15 th4)))))) (* v15 (* v4 (* 0.146003 (sin (- th15 th4))))))) (= zq_15_4 (- (- (- 0 (* (^ v15 2) 0.033082)) (* v15 (* v4 (* 0.146003 (cos (- th15 th4)))))) (* v15 (* v4 (* 0.033082 (sin (- th15 th4))))))))) (and (= zp_4_4 (* v4 (+ (+ 0 (* v15 (- (- 0 (* 0.033082 (cos (- th4 th15)))) (* 0.146003 (sin (- th4 th15)))))) (* v24 (- (- 0 (* 0.052763 (cos (- th4 th24)))) (* 0.084564 (sin (- th4 th24)))))))) (= zq_4_4 (* v4 (+ (+ 0 (* v15 (- (* 0.146003 (cos (- th4 th15))) (* 0.033082 (sin (- th4 th15)))))) (* v24 (- (* 0.084564 (cos (- th4 th24))) (* 0.052763 (sin (- th4 th24)))))))))) (and (= zp_4_4 1) (= zq_4_4 1))) (and (= zp_4_15 (- (- (* (^ v4 2) 0.033082) (* v4 (* v15 (* 0.033082 (cos (- th4 th15)))))) (* v4 (* v15 (* 0.146003 (sin (- th4 th15))))))) (= zq_4_15 (- (- (- 0 (* (^ v4 2) 0.033082)) (* v4 (* v15 (* 0.146003 (cos (- th4 th15)))))) (* v4 (* v15 (* 0.033082 (sin (- th4 th15))))))))))
(check-sat)
(exit)