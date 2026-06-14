; Plan a right-rail transport from the Staubli station to the Yaskawa station.

(define (problem room315-right-staubli-to-yaskawa)
  (:domain room315-shuttle)

  (:objects
    right - rail_side
    right_shuttle - shuttle
    right_yaskawa right_staubli - station
    right_switch_group - switch_group
    right_stopper_group - stopper_group
  )

  (:init
    (shuttle_at right_shuttle right_staubli)
    (shuttle_stopped_at right_shuttle right_staubli)
    (connected right right_staubli right_yaskawa)
  )

  (:goal
    (and
      (task_done right_shuttle right_yaskawa)
    )
  )
)
