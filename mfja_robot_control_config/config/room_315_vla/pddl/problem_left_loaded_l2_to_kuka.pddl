; Plan a left-rail transport where the loaded L2 shuttle goes to KUKA.

(define (problem room315-left-loaded-l2-to-kuka)
  (:domain room315-shuttle)

  (:objects
    left - rail_side
    left_shuttle_1 left_shuttle_2 - shuttle
    left_yaskawa left_kuka - station
    left_switch_group - switch_group
    left_stopper_group - stopper_group
  )

  (:init
    (shuttle_at left_shuttle_2 left_yaskawa)
    (shuttle_stopped_at left_shuttle_2 left_yaskawa)
    (shuttle_on_side left_shuttle_2 left)
    (loaded left_shuttle_2)
    (carrying_payload left_shuttle_2)
    (shuttle_at left_shuttle_1 left_yaskawa)
    (shuttle_stopped_at left_shuttle_1 left_yaskawa)
    (shuttle_on_side left_shuttle_1 left)
    (empty left_shuttle_1)
    (connected left left_yaskawa left_kuka)
  )

  (:goal
    (and
      (loaded left_shuttle_2)
      (task_done left_shuttle_2 left_kuka)
    )
  )
)
