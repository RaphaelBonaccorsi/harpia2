(define (problem minimal_harpia_task)
  (:domain harpia)

  (:objects
    base_1 base_2 - base
  )

  (:init
    ;; Initial state: the drone starts at base_1 with 100% battery
    (at base_1)
    (= (battery_amount) 100)
    (= (battery_capacity) 100)
    (= (discharge_rate_battery) 0.1)
    (= (velocity) 1.0)
    (= (mission_length) 0.0)
    (= (distance base_1 base_2) 10.0)
  )

  (:goal
    (at base_2)
  )
)