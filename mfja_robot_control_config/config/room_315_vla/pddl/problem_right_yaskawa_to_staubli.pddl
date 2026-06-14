; Plan a right-rail transport from the Yaskawa station to the Staubli station.

(define (problem room315-right-yaskawa-to-staubli)
  (:domain room315-shuttle)

  (:objects
    right - rail_side
    right_shuttle - shuttle
    right_yaskawa right_staubli - station
    right_switch_group - switch_group
    right_stopper_group - stopper_group
  )

  (:init
    (shuttle_at right_shuttle right_yaskawa)
    (shuttle_stopped_at right_shuttle right_yaskawa)
    (connected right right_yaskawa right_staubli)
  )

  (:goal
    (and
      (task_done right_shuttle right_staubli)
    )
  )
)
