(define (domain grid-visit-all)
(:predicates (connected ?x ?y)
	     (at-robot ?x)
	     (visited ?x)
)
	
(:action move
:parameters (?curpos ?nextpos)
:precondition (and (at-robot ?curpos) (connected ?curpos ?nextpos))
:effect (and (at-robot ?nextpos) (not (at-robot ?curpos)) (visited ?nextpos))
)

)
