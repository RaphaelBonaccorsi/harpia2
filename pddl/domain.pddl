(define (domain drone_waypoints)
  (:requirements :strips :typing :durative-actions)  ;; Adicionado :typing para suportar tipos

  ;; Definição de Tipos
  (:types 
    drone waypoint)

  ;; Predicados
  (:predicates 
    (drone_at ?d - drone ?wp - waypoint)  ;; O drone está em um waypoint específico
    (connected ?wp1 - waypoint ?wp2 - waypoint)  ;; Dois waypoints estão conectados
  )

  ;; Ação temporária de movimento
  (:durative-action move
    :parameters (?d - drone ?from - waypoint ?to - waypoint)
    :duration (= ?duration 1)  ;; Define a duração da ação
    :condition (and
      (at start (drone_at ?d ?from))
      (over all (connected ?from ?to))
    )
    :effect (and
      (at start (not (drone_at ?d ?from)))
      (at end (drone_at ?d ?to))
    )
  )
)