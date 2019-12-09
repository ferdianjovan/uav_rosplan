(define (domain uas)

    (:requirements 
        :strips
        :typing
        :equality
        :fluents
        :durative-actions
        :duration-inequalities
        :continuous-effects
    )

    (:types
        waypoint robot - object
        uav asv - robot
    )

    (:functions
        (battery-amount ?r - robot)
        (minimum-battery ?r - robot)
    )

    (:predicates
        (at ?r - robot ?wp - waypoint)
        (visited ?r - robot ?wp - waypoint)
        (home ?wp - waypoint)
        (airborne ?v - uav)
        (landed ?v - uav)
        (armed ?v - uav)
        (guided ?v - uav)
        (preflightchecked ?v - uav)
    )

    (:durative-action preflightcheck
        :parameters  (?v - uav)
        :duration (<= ?duration 300)
        :condition (and (over all (landed ?v)))
        :effect (and (at end (preflightchecked ?v)))
    )

    (:durative-action guide_mode
        :parameters (?v - uav)
        :duration (<= ?duration 60)
        :condition (and (over all (preflightchecked ?v)))
        :effect (and (at end (guided ?v)))
    )

    (:durative-action request_arm
        :parameters (?v - uav)
        :duration (<= ?duration 300)
        :condition (and 
            (at start (guided ?v))
            (over all (landed ?v))
            (over all (preflightchecked ?v))
        )
        :effect (and (at end (armed ?v)))
    )   

    (:durative-action takeoff
        :parameters (?v - uav)
        :duration (<= ?duration 60)
        :condition (and 
            (at start (landed ?v))
            (at start (> (battery-amount ?v) (minimum-battery ?v)))
            (over all (armed ?v))
            (over all (guided ?v))
            (over all (preflightchecked ?v))
        )
        :effect (and 
            (at start (not (landed ?v)))
            (at end (airborne ?v))
            (decrease (battery-amount ?v) (* 0.001 #t))
        )
    )

    ;; Move to any waypoint on a straight line
    (:durative-action goto_waypoint
        :parameters (?v - uav ?from ?to - waypoint)
        :duration (<= ?duration 120)
        :condition (and
            (at start (at ?v ?from))
            (at start (> (battery-amount ?v) (minimum-battery ?v)))
            (over all (armed ?v))
            (over all (airborne ?v))
            (over all (preflightchecked ?v))
        )
        :effect (and
            (at start (not (at ?v ?from)))
            (at end (at ?v ?to))
            (at end (visited ?v ?to))
            (decrease (battery-amount ?v) (* 0.001 #t))
        )
    )

    (:durative-action rtl
        :parameters (?v - uav ?from ?to - waypoint)
        :duration (<= ?duration 240)
        :condition (and
            (at start (airborne ?v))
            (at start (at ?v ?from))
            (over all (home ?to))
            (over all (armed ?v))
            (over all (preflightchecked ?v))
        )
        :effect (and 
            (at end (landed ?v))
            (at end (at ?v ?to))
            (at end (visited ?v ?to))
            (at start (not (at ?v ?from)))
            (at start (not (airborne ?v)))
            (decrease (battery-amount ?v) (* 0.001 #t))
        )
    )

    (:durative-action lowbat_return
        :parameters (?v - uav ?from ?to - waypoint)
        :duration (<= ?duration 240)
        :condition (and
            (over all (home ?to))
            (over all (armed ?v))
            (at start (airborne ?v))
            (at start (at ?v ?from))
            (at start (<= (battery-amount ?v) (minimum-battery ?v)))
        )
        :effect (and 
            (at end (landed ?v))
            (at end (at ?v ?to))
            (at start (not (at ?v ?from)))
            (at start (not (airborne ?v)))
            (decrease (battery-amount ?v) (* 0.001 #t))
        )
    )

)
