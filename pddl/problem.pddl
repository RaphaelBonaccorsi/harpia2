(define (problem drone_waypoint_following)
  (:domain drone_waypoints)

  ;; Definição de objetos
  (:objects 
    drone1 - drone
    waypoint_1 waypoint_2 waypoint_3 waypoint_4 waypoint_5 - waypoint
  )

  ;; Estado inicial
  (:init
    (drone_at drone1 waypoint_1)
    (connected waypoint_1 waypoint_2)
    (connected waypoint_2 waypoint_3)
    (connected waypoint_3 waypoint_4)
    (connected waypoint_4 waypoint_5)
  )

  ;; Estado objetivo
  (:goal (and (drone_at drone1 waypoint_5)))
)
