(define (problem simple-harpia-task)
  (:domain harpia)

  (:objects
    region_1 region_2 - region
    base_1 - base
  )

  (:init
    ;; Estado inicial: drone está na base com uma bateria de 80% e sem insumo
    (at base_1)
    (= (battery-amount) 80)
    (= (input-amount) 0)
    (= (battery-capacity) 100)
    (= (input-capacity) 3)
    (= (discharge-rate-battery) 0.1)
    (= (velocity) 2.0)
    (= (distance base_1 region_1) 100.0)
    (= (distance region_1 region_2) 50.0)

    ;; Definindo o objetivo de imagem para a região 1
    (picture-goal region_1)
  )

  (:goal
    (and
      (taken-image region_1)   ;; O objetivo é tirar uma imagem em region_1
      (at region_2)            ;; E terminar na região 2
    )
  )
  
  (:metric minimize (mission-length))
)
