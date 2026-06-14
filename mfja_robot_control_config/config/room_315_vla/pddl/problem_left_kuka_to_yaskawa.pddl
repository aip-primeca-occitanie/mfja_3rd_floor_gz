; Plan a left-rail transport from the KUKA station to the Yaskawa station.

(define (problem room315-left-kuka-to-yaskawa)
  (:domain room315-shuttle)

  (:objects
    left - rail_side
    left_shuttle - shuttle
    left_yaskawa left_kuka - station
    left_switch_group - switch_group
    left_stopper_group - stopper_group
  )

  (:init
    (shuttle_at left_shuttle left_kuka)
    (shuttle_stopped_at left_shuttle left_kuka)
    (connected left left_kuka left_yaskawa)
  )

  (:goal
    (and
      (task_done left_shuttle left_yaskawa)
    )
  )
)
