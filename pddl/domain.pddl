(define (domain uas)

    (:requirements
        :strips
        :typing
    )

    (:types
        waypoint robot - object
        uav asv - robot
    )

    (:predicates
        (at ?r - robot ?wp - waypoint)
        (visited ?r - robot ?wp - waypoint)
        (home ?wp - waypoint)
        (airborne ?v - uav)
        (landed ?v - uav)
        (armed ?v - uav)
        (guided ?v - uav)
        (lowbat ?v - uav)
        (preflightchecked ?v - uav)
    )

    (:action preflightcheck
        :parameters  (?v - uav)
        :precondition (and (landed ?v)
            ; (not (airborne ?v))
            ; (not (armed ?v))
        )
        :effect (and (preflightchecked ?v))
    )
    
    (:action guide_mode
        :parameters (?v - uav)
        :precondition (and (preflightchecked ?v))
        :effect (and (guided ?v))
    )

    (:action request_arm
        :parameters (?v - uav)
        :precondition (and (landed ?v)
            (guided ?v)
            (preflightchecked ?v)
            ; (not (airborne ?v))
        )
        :effect (and (armed ?v))
    )   

    (:action takeoff
        :parameters (?v - uav)
        :precondition (and (landed ?v)
            (guided ?v)
            (armed ?v)
            (preflightchecked ?v)
            ; (not (lowbat ?v))
            ; (not (airborne ?v))
        )
        :effect (and (not (landed ?v)) (airborne ?v))
    )

    ;; Move to any waypoint on a straight line
    (:action goto_waypoint
        :parameters (?v - uav ?from ?to - waypoint)
        :precondition (and (preflightchecked ?v)
            (armed ?v)
            (airborne ?v)
            (at ?v ?from)
            ; (not (lowbat ?v))
            ; (not (landed ?v))
        )
        :effect (and (at ?v ?to)
            (visited ?v ?to)
            (not (at ?v ?from))
        )
    )

    (:action rtl
        :parameters (?v - uav ?from ?to - waypoint)
        :precondition (and (airborne ?v)
            (preflightchecked ?v)
            (armed ?v)
            (at ?v ?from)
            (home ?to)
        )
        :effect (and (landed ?v)
            (at ?v ?to)
            (visited ?v ?to)
            (not (at ?v ?from))
            (not (airborne ?v))
        )
    )

    (:action lowbat_return
        :parameters (?v - uav ?from ?to - waypoint)
        :precondition (and (lowbat ?v)
            (airborne ?v)
            (armed ?v)
            (at ?v ?from)
            (home ?to)
        )
        :effect (and (landed ?v)
            (at ?v ?to)
            (visited ?v ?to)
            (not (at ?v ?from))
            (not (airborne ?v))
        )
    )

)
