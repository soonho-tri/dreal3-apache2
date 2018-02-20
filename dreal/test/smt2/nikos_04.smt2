(set-logic QF_NRA)
(set-option :precision 0.000001)
(set-option :polytope true)
(declare-fun x1 () Real)
(declare-fun x2 () Real)
(declare-fun x3 () Real)
(declare-fun x4 () Real)

(assert (> x1 -0.2))
(assert (< x1 0.2))
(assert (> x2 -0.2))
(assert (< x2 0.2))
(assert (> x3 -0.2))
(assert (< x3 0.2))
(assert (> x4 -0.2))
(assert (< x4 0.2))

(assert (< (+ (* x1 x1) (* x2 x2) (* x3 x3) (* x4 x4)) 0.0004))

(assert (> (+   (* 99.93480571  x1 x1)
                (* 30.33988648  x1 x2)
                (* (- 102.36911255) x1 x3)
                (* 12.23908749  x1 x4)
                (* 112.06951340 x1 (* x1 x1))
                (* 53.98882124  x1 (* x1 x2))
                (* 104.85634611 x1 (* x2 x2))
                (* 90.39545543  x1 (* x1 x3))
                (* 4.76548912   x1 (* x2 x3))
                (* 128.60133992 x1 (* x3 x3))
                (* 11.59236537  x1 (* x1 x4))
                (* (- 11.88723162 ) x1 (* x2 x4))
                (* 3.17269161   x1 (* x3 x4))
                (* (- 41.43224247 ) x1 (* x4 x4))
                (* 94.49599160  x2 x2)
                (* (- 6.56262926  ) x2 x3)
                (* 124.40733337 x2 x4)
                (* 110.03260351 x2 (* x1 x1))
                (* 160.26070647 x2 (* x1 x2))
                (* (- 112.82732678) x2 (* x2 x2))
                (* (- 17.64701753 ) x2 (* x1 x3))
                (* 33.14166757  x2 (* x2 x3))
                (* 49.21135585  x2 (* x3 x3))
                (* (- 62.31715698 ) x2 (* x1 x4))
                (* 155.37080703 x2 (* x2 x4))
                (* 149.04126317 x2 (* x3 x4))
                (* 118.17098043 x2 (* x4 x4))
                (* 78.39144967  x3 x3)
                (* 17.91122227  x3 x4)
                (* 27.14962222  x3 (* x1 x1))
                (* (- 61.25979053 ) x3 (* x1 x2))
                (* (- 12.64530368 ) x3 (* x2 x2))
                (* 22.25713016  x3 (* x1 x3))
                (* 5.06103276   x3 (* x2 x3))
                (* (- 29.56775941 ) x3 (* x3 x3))
                (* 111.84648585 x3 (* x1 x4))
                (* 19.03875013  x3 (* x2 x4))
                (* (- 39.31429428 ) x3 (* x3 x4))
                (* 78.07916558  x3 (* x4 x4))
                (* 99.90378402  x4 x4)
                (* 67.58958775  x4 (* x1 x1))
                (* 39.96934692  x4 (* x1 x2))
                (* (- 50.76131473 ) x4 (* x2 x2))
                (* (- 40.65684216 ) x4 (* x1 x3))
                (* 21.01300664  x4 (* x2 x3))
                (* (- 9.20415544  ) x4 (* x3 x3))
                (* 18.76615721  x4 (* x1 x4))
                (* 123.17515793 x4 (* x2 x4))
                (* 149.49396115 x4 (* x3 x4))
                (* 9.58029820   x4 (* x4 x4))
                (* 96.98046181  (* x1 x1) (* x1 x1))
                (* 86.26931608  (* x1 x1) (* x1 x2))
                (* 11.70584449  (* x1 x1) (* x2 x2))
                (* 88.05269782  (* x1 x1) (* x1 x3))
                (* 19.50487392  (* x1 x1) (* x2 x3))
                (* 130.61630834 (* x1 x1) (* x3 x3))
                (* 61.46239621  (* x1 x1) (* x1 x4))
                (* 84.82402633  (* x1 x1) (* x2 x4))
                (* 47.49059545  (* x1 x1) (* x3 x4))
                (* 102.39369206 (* x1 x1) (* x4 x4))
                (* 96.19286486  (* x1 x2) (* x1 x2))
                (* (- 111.75991382) (* x1 x2) (* x2 x2))
                (* (- 12.29053285 ) (* x1 x2) (* x1 x3))
                (* 26.51188564  (* x1 x2) (* x2 x3))
                (* 71.25939413  (* x1 x2) (* x3 x3))
                (* (- 118.26497966) (* x1 x2) (* x1 x4))
                (* 119.42455446 (* x1 x2) (* x2 x4))
                (* 117.85584494 (* x1 x2) (* x3 x4))
                (* 115.95639010 (* x1 x2) (* x4 x4))
                (* 98.84561500  (* x2 x2) (* x2 x2))
                (* 103.09289606 (* x2 x2) (* x1 x3))
                (* (- 10.39592531 ) (* x2 x2) (* x2 x3))
                (* 49.36208784  (* x2 x2) (* x3 x3))
                (* 120.36280034 (* x2 x2) (* x1 x4))
                (* (- 126.41483400) (* x2 x2) (* x2 x4))
                (* (- 132.30287845) (* x2 x2) (* x3 x4))
                (* (- 120.88926538) (* x2 x2) (* x4 x4))
                (* 59.95019505  (* x1 x3) (* x1 x3))
                (* 2.90167800   (* x1 x3) (* x2 x3))
                (* 88.13882195  (* x1 x3) (* x3 x3))
                (* 89.75457764  (* x1 x3) (* x1 x4))
                (* (- 38.82077374 ) (* x1 x3) (* x2 x4))
                (* (- 84.52342357 ) (* x1 x3) (* x3 x4))
                (* 15.28362811  (* x1 x3) (* x4 x4))
                (* 6.54968160   (* x2 x3) (* x2 x3))
                (* 11.70034286  (* x2 x3) (* x3 x3))
                (* (- 0.83321450  ) (* x2 x3) (* x1 x4))
                (* 24.82532627  (* x2 x3) (* x2 x4))
                (* 16.67487592  (* x2 x3) (* x3 x4))
                (* 20.36353067  (* x2 x3) (* x4 x4))
                (* 65.29014889  (* x3 x3) (* x3 x3))
                (* 26.84494489  (* x3 x3) (* x1 x4))
                (* 18.70852843  (* x3 x3) (* x2 x4))
                (* (- 7.68874881  ) (* x3 x3) (* x3 x4))
                (* 47.40079412  (* x3 x3) (* x4 x4))
                (* 97.17953398  (* x1 x4) (* x1 x4))
                (* (- 48.83097933 ) (* x1 x4) (* x2 x4))
                (* (- 93.28360889 ) (* x1 x4) (* x3 x4))
                (* (- 17.18972007 ) (* x1 x4) (* x4 x4))
                (* 80.48480640  (* x2 x4) (* x2 x4))
                (* 148.34930406 (* x2 x4) (* x3 x4))
                (* 118.69942346 (* x2 x4) (* x4 x4))
                (* 97.96466358  (* x3 x4) (* x3 x4))
                (* 56.58992087  (* x3 x4) (* x4 x4))
                (* 99.24526667  (* x4 x4) (* x4 x4))
            )
            0.07
    )
)

(check-sat)
(exit)
