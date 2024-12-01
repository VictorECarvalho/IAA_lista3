(define (problem grid-2-grounded)
(:domain grid-visit-all-grounded-for-grid-2)
(:objects
)
(:init
   (at-robot-loc-x2-y0)
   (visited-loc-x2-y0)
)
(:goal
    (and
        (visited-loc-x0-y0)
        (visited-loc-x1-y0)
        (visited-loc-x2-y0)
        (visited-loc-x3-y0)
        (visited-loc-x4-y0)
    )
)
)
