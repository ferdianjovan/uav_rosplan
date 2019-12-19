(define (problem uas_mission)

    (:domain uas)

    (:objects
        wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
        hector - uav
    )

    (:init
        (at hector wp0)
        (landed hector)
        (home wp0)
        (visited hector wp0)
        (= (battery-amount hector) 12.59)
        (= (minimum-battery hector) 12.19)
    )

    (:goal (or
        (and (visited hector wp1)
            (visited hector wp2)
            (visited hector wp3)
            (visited hector wp4)
            (visited hector wp5)
            (landed hector)
        )
        (and (landed hector)
            (<= (battery-amount hector) (minimum-battery hector))
        )
    ))

    (:metric minimize (total-time))

)
