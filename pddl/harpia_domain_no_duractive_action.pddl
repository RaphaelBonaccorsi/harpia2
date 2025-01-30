(define (domain harpia)

    (:requirements  :typing  :strips  :disjunctive-preconditions  :equality :fluents )

    (:types
        region - object
        base - region)


   
    (:functions
    

        ;; Variavel q controla bateria em porcentagem
        (battery_amount)
        ;; quantidade de insumo
        (input_amount)
        ;;velocidade de carregar a bateria em porcentagem por segundos
        (recharge_rate_battery)
        ;;velocidade de descarregar a bateria
        (discharge_rate_battery)
        ;;capacidade maxima bateria
        (battery_capacity)
        ;;capacidade maxima de insumo
        (input_capacity)
        ;;velocidade de reabastecer o insumo
        (recharge_rate_input)
        ;;distancia entre regioes em metros
        (distance ?from_region - region ?to_region - region)
        ;;velocidade em m/s
        (velocity)
        (picture_path_len ?region - region)
        (pulverize_path_len ?region - region)
        (total_goals)
        (goals_achived)
        (mission_length)

    )

     (:predicates
    
        (been_at ?region - region)
        ;;se esta carregando um insumo
        (carry)  
        ;;esta em uma regiao
        (at ?region - region)
        ;; se pode pulverizar
        (can_spray)
        ;;se pode carregar/descarregar
        (can_recharge)
        ;se já tirou a foto
        (taken_image ?region - region)
        ;se pulverizou
        (pulverized ?region - region)
        ; (canGo)
        (can_take_pic)
        (its_not_base ?region - region)
        (pulverize_goal ?region - region)
        (picture_goal ?region - region)
        (hw_ready ?from - region ?to - region)

        ; (can_go_to_base)
        ; (has_pulverize_goal)
        ; (has_picture_goal)
        ; (at_move)
    
    )


    (:action go_to
        :parameters (
             ?from_region - region 
             ?to_region - region)
        :precondition (and
            (at ?from_region)
            (> (battery_amount) (+ (* (/ (distance ?from_region ?to_region) (velocity)) (discharge_rate_battery)) 15))
        )
        :effect (and 
                (not (at ?from_region))
                (been_at ?to_region)
                (at ?to_region)
                (decrease (battery_amount ) 
                      (*
                          (/
                              (distance ?from_region ?to_region)
                              (velocity)
                          )
                          (discharge_rate_battery)
                      )
          
                )
                (increase (mission_length) (distance ?from_region ?to_region))
                )
    )
    
    (:action take_image
        :parameters (
            ?region - region
        )
        :precondition(and
            (at ?region)
            (picture_goal ?region)
            (> (battery_amount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (discharge_rate_battery)
                )
            )
       )
        :effect(and
            (taken_image ?region)
            (increase (mission_length) 1000)
            (decrease (battery_amount) 
                (*
                    (/
                        1000
                        (velocity)
                    )
                    (discharge_rate_battery)
                )
            )
        )
    )
    (:action pulverize_region
        :parameters (
            ?region - region)
        :precondition(and
            (at ?region)
            (pulverize_goal ?region)
            (> (input_amount) 0)
            (> (battery_amount) 
                (*
                    (/
                        314
                        (velocity)
                    )
                    (discharge_rate_battery)
                )
            )
       )
        :effect(and
            (pulverized ?region)
            (increase (mission_length) 314)
            (decrease (input_amount) 1)
            (decrease (battery_amount) 
                (*
                    (/
                        314
                        (velocity)
                    )
                    (discharge_rate_battery)
                )
            )
        )
    )
    (:action recharge_battery
        :parameters (?base - base)
        :precondition (and
            (at ?base)
            ;(< (battery_amount) 60)
        )
        :effect 
        (and
            (assign (battery_amount) (battery_capacity))
        )
    )

    (:action recharge_input
        :parameters (?base - base)
        :precondition (and
            (at ?base)
            (< (input_amount) (/ (input_capacity) 2))
        )
        :effect 
        (and
            (assign (input_amount) (input_capacity))
        )
    )
)