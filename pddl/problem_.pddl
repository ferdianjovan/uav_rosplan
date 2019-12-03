(define (problem uas_mission)

    (:domain uas)

    (:objects
        wp0 wp1 wp2 wp3 wp4 wp5 - waypoint
        hector - uav
    )

    (:init
        (uav_at hector wp0)
        (landed hector)
        (home wp0)
        (visited wp0)
    )

    (:goal (and
        (visited wp1)
        (visited wp2)
        (visited wp3)
        (visited wp4)
        (visited wp5)
        (landed hector)
    ))

)
