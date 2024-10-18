(define (problem minimal-harpia-task)
  (:domain harpia)

  (:objects
    base_1 base_2 - base
  )

  (:init
    ;; Initial state: the drone starts at base_1 with 100% battery
    (at base_1)
    (= (battery-amount) 100)
    (= (battery-capacity) 100)
    (= (discharge-rate-battery) 0.1)
    (= (velocity) 1.0)
    (= (distance base_1 base_2) 10.0)
  )

  (:goal
    (at base_2)  ;; The goal is to move to base_2
  )
)