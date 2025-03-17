(define (problem drone_waypoint_following)
  (:domain drone_waypoints)

  ;; Objects (instances of types)
  (:objects 
    drone1 - drone
    region_1 region_2 region_3 region_4 region_5 - waypoint
  )

  ;; Initial state
  (:init
    (drone_at drone1 region_1)
    (connected region_1 region_2)
    (connected region_2 region_3)
    (connected region_3 region_4)
    (connected region_4 region_5)
  )

  ;; Goal state
  (:goal (drone_at drone1 region_5))
)