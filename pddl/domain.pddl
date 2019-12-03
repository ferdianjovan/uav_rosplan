(define (domain uas)

    (:requirements
        :strips
        :typing
    )

    (:types
        uav waypoint - object
    )

    (:predicates
        (uav_at ?v - uav ?wp - waypoint)
        (visited ?wp - waypoint)
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
    
    (:action request_arm
        :parameters (?v - uav)
        :precondition (and (landed ?v)
            (guided ?v)
            (preflightchecked ?v)
            ; (not (airborne ?v))
        )
        :effect (and (armed ?v))
    )   

    (:action guide_mode
        :parameters (?v - uav)
        :precondition (and (preflightchecked ?v))
        :effect (and (guided ?v))
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
            (uav_at ?v ?from)
            ; (not (lowbat ?v))
            ; (not (landed ?v))
        )
        :effect (and (visited ?to)
            (uav_at ?v ?to)
            (not (uav_at ?v ?from))
        )
    )

    (:action rtl
        :parameters (?v - uav ?from ?to - waypoint)
        :precondition (and (airborne ?v)
            (preflightchecked ?v)
            (armed ?v)
            (uav_at ?v ?from)
            (home ?to)
        )
        :effect (and (landed ?v)
            (uav_at ?v ?to)
            (not (uav_at ?v ?from))
            (not (airborne ?v))
        )
    )

)
