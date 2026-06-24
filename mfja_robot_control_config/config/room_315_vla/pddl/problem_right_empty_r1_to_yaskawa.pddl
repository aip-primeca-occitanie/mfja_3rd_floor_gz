; Plan a right-rail transport where the empty R1 shuttle goes to Yaskawa.

(define (problem room315-right-empty-r1-to-yaskawa)
  (:domain room315-shuttle)

  (:objects
    right - rail_side
    right_shuttle_1 right_shuttle_2 - shuttle
    right_yaskawa right_staubli - station
    right_switch_group - switch_group
    right_stopper_group - stopper_group
  )

  (:init
    (shuttle_at right_shuttle_1 right_staubli)
    (shuttle_stopped_at right_shuttle_1 right_staubli)
    (shuttle_on_side right_shuttle_1 right)
    (empty right_shuttle_1)
    (shuttle_at right_shuttle_2 right_staubli)
    (shuttle_stopped_at right_shuttle_2 right_staubli)
    (shuttle_on_side right_shuttle_2 right)
    (loaded right_shuttle_2)
    (carrying_payload right_shuttle_2)
    (connected right right_staubli right_yaskawa)
  )

  (:goal
    (and
      (empty right_shuttle_1)
      (task_done right_shuttle_1 right_yaskawa)
    )
  )
)
