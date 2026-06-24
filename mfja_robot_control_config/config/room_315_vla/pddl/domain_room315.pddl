; Room 315 high-level shuttle planning domain.
;
; This first PDDL milestone is intentionally symbolic. It plans station-to-
; station shuttle scenarios only. Detailed switch A1..A4, stopper A1..A4,
; safety checks, action_vector encoding, and ROS command execution remain in the
; existing VLA supervisor and are not modeled here.

(define (domain room315-shuttle)
  (:requirements :strips :typing :negative-preconditions)

  (:types
    rail_side
    shuttle
    station
    slot
    block
    switch_group
    stopper_group
    payload
  )

  (:predicates
    ; The shuttle is currently located at a named station.
    (shuttle_at ?s - shuttle ?station - station)

    ; The shuttle is stopped at a named station. This makes stop_shuttle a
    ; required symbolic step before finish_task can complete.
    (shuttle_stopped_at ?s - shuttle ?station - station)

    ; The high-level path for a side and station pair has been prepared.
    (path_ready ?side - rail_side ?from - station ?to - station)

    ; The switches for this side have been prepared at the coarse group level.
    (switches_ready ?side - rail_side)

    ; The stoppers for this side are open at the coarse group level.
    (stoppers_open ?side - rail_side)

    ; The two stations are connected on the given rail side.
    (connected ?side - rail_side ?from - station ?to - station)

    ; The requested shuttle task has finished at the target station.
    (task_done ?s - shuttle ?station - station)

    ; Multi-shuttle extensions: identity, slot/block occupancy, reservations,
    ; payload state, and relative ordering. These are planner/supervisor
    ; metadata only and are not model_input.
    (shuttle_on_side ?s - shuttle ?side - rail_side)
    (shuttle_at_slot ?s - shuttle ?slot - slot)
    (shuttle_at_block ?s - shuttle ?block - block)
    (block_free ?block - block)
    (block_reserved_by ?block - block ?s - shuttle)
    (slot_free ?slot - slot)
    (slot_reserved_by ?slot - slot ?s - shuttle)
    (loaded ?s - shuttle)
    (empty ?s - shuttle)
    (carrying_payload ?s - shuttle)
    (payload_on_shuttle ?p - payload ?s - shuttle)
    (route_clear ?s - shuttle ?from - station ?to - station)
    (switches_ready_for ?s - shuttle)
    (stoppers_open_for ?s - shuttle)
    (task_assigned ?s - shuttle ?station - station)
    (front_of ?front - shuttle ?rear - shuttle)
    (behind ?rear - shuttle ?front - shuttle)
    (waiting_for_clearance ?s - shuttle)
  )

  ; Prepare the switch group for the route. This is a high-level placeholder
  ; for future mapping to route templates or primitive switch commands.
  (:action prepare_switches
    :parameters (
      ?side - rail_side
      ?from - station
      ?to - station
      ?switches - switch_group
    )
    :precondition (connected ?side ?from ?to)
    :effect (switches_ready ?side)
  )

  ; Open the stopper group after switches are prepared. Once both switch and
  ; stopper groups are ready, the symbolic path is considered ready.
  (:action open_stoppers
    :parameters (
      ?side - rail_side
      ?from - station
      ?to - station
      ?stoppers - stopper_group
    )
    :precondition (and
      (connected ?side ?from ?to)
      (switches_ready ?side)
    )
    :effect (and
      (stoppers_open ?side)
      (path_ready ?side ?from ?to)
    )
  )

  ; Move the shuttle along a prepared high-level path. This does not encode
  ; low-level speed, wait condition, or safety constraints yet.
  (:action move_shuttle
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from - station
      ?to - station
    )
    :precondition (and
      (shuttle_at ?s ?from)
      (connected ?side ?from ?to)
      (path_ready ?side ?from ?to)
      (switches_ready ?side)
      (stoppers_open ?side)
    )
    :effect (and
      (not (shuttle_at ?s ?from))
      (not (shuttle_stopped_at ?s ?from))
      (shuttle_at ?s ?to)
    )
  )

  ; Stop the shuttle after it arrives at the target station and clear the
  ; symbolic active path. Runtime stopping will remain supervisor controlled.
  (:action stop_shuttle
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from - station
      ?to - station
    )
    :precondition (and
      (shuttle_at ?s ?to)
      (path_ready ?side ?from ?to)
    )
    :effect (and
      (shuttle_stopped_at ?s ?to)
      (not (path_ready ?side ?from ?to))
    )
  )

  ; Mark the task complete once the shuttle is stopped at the target station.
  ; This is the high-level terminal marker for generated scenarios.
  (:action finish_task
    :parameters (
      ?s - shuttle
      ?station - station
    )
    :precondition (and
      (shuttle_at ?s ?station)
      (shuttle_stopped_at ?s ?station)
    )
    :effect (task_done ?s ?station)
  )

  ; Assign a station target to a specific shuttle. This makes target identity
  ; explicit for multi-shuttle PlanSys2 scenarios.
  (:action assign_task
    :parameters (
      ?s - shuttle
      ?station - station
    )
    :precondition (not (task_done ?s ?station))
    :effect (task_assigned ?s ?station)
  )

  ; Reserve the next symbolic rail block for one shuttle.
  (:action reserve_next_block
    :parameters (
      ?s - shuttle
      ?from - block
      ?to - block
    )
    :precondition (and
      (shuttle_at_block ?s ?from)
      (block_free ?to)
    )
    :effect (and
      (block_reserved_by ?to ?s)
      (not (block_free ?to))
    )
  )

  ; Release a symbolic rail block after the shuttle leaves it.
  (:action release_block
    :parameters (
      ?s - shuttle
      ?block - block
    )
    :precondition (block_reserved_by ?block ?s)
    :effect (and
      (block_free ?block)
      (not (block_reserved_by ?block ?s))
    )
  )

  ; Prepare switches for one shuttle, keeping identity in the symbolic plan.
  (:action prepare_switches_for_shuttle
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from - station
      ?to - station
      ?switches - switch_group
    )
    :precondition (and
      (shuttle_on_side ?s ?side)
      (connected ?side ?from ?to)
    )
    :effect (switches_ready_for ?s)
  )

  ; Open stoppers for one shuttle after its switches are ready.
  (:action open_stoppers_for_shuttle
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from - station
      ?to - station
      ?stoppers - stopper_group
    )
    :precondition (and
      (shuttle_on_side ?s ?side)
      (switches_ready_for ?s)
    )
    :effect (and
      (stoppers_open_for ?s)
      (route_clear ?s ?from ?to)
    )
  )

  ; Move one shuttle from a reserved block into the next block.
  (:action move_shuttle_to_block
    :parameters (
      ?s - shuttle
      ?from - block
      ?to - block
    )
    :precondition (and
      (shuttle_at_block ?s ?from)
      (block_reserved_by ?to ?s)
    )
    :effect (and
      (not (shuttle_at_block ?s ?from))
      (shuttle_at_block ?s ?to)
      (block_free ?from)
    )
  )

  ; Stop a shuttle at a named station slot.
  (:action stop_shuttle_at_slot
    :parameters (
      ?s - shuttle
      ?slot - slot
    )
    :precondition (slot_reserved_by ?slot ?s)
    :effect (and
      (shuttle_at_slot ?s ?slot)
      (not (slot_reserved_by ?slot ?s))
    )
  )

  ; Explicit wait action used when a block/slot is not yet clear.
  (:action wait_for_clearance
    :parameters (
      ?s - shuttle
      ?block - block
    )
    :precondition (not (block_free ?block))
    :effect (waiting_for_clearance ?s)
  )

  ; Payload handoff placeholder for future robot/rail integration.
  (:action transfer_payload_if_applicable
    :parameters (
      ?p - payload
      ?from - shuttle
      ?to - shuttle
    )
    :precondition (payload_on_shuttle ?p ?from)
    :effect (and
      (not (payload_on_shuttle ?p ?from))
      (payload_on_shuttle ?p ?to)
      (not (loaded ?from))
      (empty ?from)
      (not (empty ?to))
      (loaded ?to)
      (not (carrying_payload ?from))
      (carrying_payload ?to)
    )
  )
)
