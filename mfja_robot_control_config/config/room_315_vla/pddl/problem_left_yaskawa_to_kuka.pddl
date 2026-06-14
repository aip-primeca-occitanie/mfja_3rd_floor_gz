; Plan a left-rail transport from the Yaskawa station to the KUKA station.

(define (problem room315-left-yaskawa-to-kuka)
  (:domain room315-shuttle)

  (:objects
    left - rail_side
    left_shuttle - shuttle
    left_yaskawa left_kuka - station
    left_switch_group - switch_group
    left_stopper_group - stopper_group
  )

  (:init
    (shuttle_at left_shuttle left_yaskawa)
    (shuttle_stopped_at left_shuttle left_yaskawa)
    (connected left left_yaskawa left_kuka)
  )

  (:goal
    (and
      (task_done left_shuttle left_kuka)
    )
  )
)
