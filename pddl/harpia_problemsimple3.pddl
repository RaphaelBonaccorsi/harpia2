(define (problem elaborated_harpia_task)
  (:domain harpia)

  (:objects
    region_1 region_2 - region
    base_1 - base
  )

  (:init
    ;; Initial state: drone starts at base_1 with some battery charge
    (at base_1)
    (= (battery_amount) 50)         ;; 50% battery at the start
    (= (battery_capacity) 100)      ;; Max battery capacity
    (= (input_amount) 1)            ;; 1 unit of input available
    (= (input_capacity) 3)          ;; Max input capacity
    (= (discharge_rate_battery) 0.04) ;; Battery drains at 0.04 per meter
    (= (velocity) 1.0)              ;; Moving at 1 m/s
    (= (distance base_1 region_1) 10.0)
    (= (distance region_1 region_2) 20.0)

    ;; Picture and pulverize goals for the regions
    (picture_goal region_1)
    (pulverize_goal region_2)
  )

  (:goal
    (and
      (taken_image region_1)    ;; Picture taken at region_1
      (pulverized region_2)     ;; Region_2 has been pulverized
      (at base_1)               ;; Drone returns to base_1
    )
  )

  (:metric minimize (mission_length))
)
