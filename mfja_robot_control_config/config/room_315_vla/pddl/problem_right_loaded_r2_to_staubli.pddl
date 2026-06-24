; Plan a right-rail transport where the loaded R2 shuttle goes to Staubli.

(define (problem room315-right-loaded-r2-to-staubli)
  (:domain room315-shuttle)

  (:objects
    right - rail_side
    right_shuttle_1 right_shuttle_2 - shuttle
    right_yaskawa right_staubli - station
    right_switch_group - switch_group
    right_stopper_group - stopper_group
  )

  (:init
    (shuttle_at right_shuttle_2 right_yaskawa)
    (shuttle_stopped_at right_shuttle_2 right_yaskawa)
    (shuttle_on_side right_shuttle_2 right)
    (loaded right_shuttle_2)
    (carrying_payload right_shuttle_2)
    (shuttle_at right_shuttle_1 right_yaskawa)
    (shuttle_stopped_at right_shuttle_1 right_yaskawa)
    (shuttle_on_side right_shuttle_1 right)
    (empty right_shuttle_1)
    (connected right right_yaskawa right_staubli)
  )

  (:goal
    (and
      (loaded right_shuttle_2)
      (task_done right_shuttle_2 right_staubli)
    )
  )
)
