; Room 315 production runtime planning domain.
;
; This is the POPF-safe execution subset. Dataset/scenario tooling keeps the
; broader expert domain in domain_room315.pddl, while the live gateway accepts
; explicit transport goals and executes only these supervised primitives.

(define (domain room315-shuttle)
  (:requirements :strips :typing :negative-preconditions :fluents)

  (:types
    inspection_target rail_side shuttle station slot block
    switch_device stopper_device switch_group stopper_group obstacle
  )

  (:predicates
    (validated_state)
    (observation_required)
    (shuttle_on_side ?s - shuttle ?side - rail_side)
    (shuttle_at ?s - shuttle ?station - station)
    (shuttle_stopped_at ?s - shuttle ?station - station)
    (shuttle_at_slot ?s - shuttle ?slot - slot)
    (shuttle_in_block ?s - shuttle ?block - block)
    (shuttle_at_topology_block ?s - shuttle ?block - block)
    (loaded ?s - shuttle)
    (empty ?s - shuttle)
    (slot_on_side ?slot - slot ?side - rail_side)
    (slot_at_station ?slot - slot ?station - station)
    (slot_in_block ?slot - slot ?block - block)
    (slot_free ?slot - slot)
    (slot_occupied_by ?slot - slot ?s - shuttle)
    (slot_reserved_by ?slot - slot ?s - shuttle)
    (block_on_side ?block - block ?side - rail_side)
    (block_free ?block - block)
    (block_occupied_by ?block - block ?s - shuttle)
    (block_reserved_by ?block - block ?s - shuttle)
    (connected ?side - rail_side ?from - station ?to - station)
    (path_ready ?side - rail_side ?from - station ?to - station)
    (route_clear_between ?from - slot ?to - slot)
    (route_blocked_by ?from - slot ?to - slot ?blocker - shuttle)
    (topology_route_available ?s - shuttle ?from - block ?to - slot)
    (topology_route_clear ?s - shuttle ?from - block ?to - slot)
    (topology_route_blocked_by ?s - shuttle ?from - block ?to - slot ?blocker - shuttle)
    (topology_route_configured ?s - shuttle ?from - block ?to - slot)
    (clearance_precedes ?blocker - shuttle ?selected - shuttle)
    (clearance_relocated ?blocker - shuttle)
    (clearance_destination_ready ?blocker - shuttle)
    (normal_route ?side - rail_side)
    (clearance_mode ?side - rail_side)
    (route_reserved_by ?from - slot ?to - slot ?s - shuttle)
    (switch_state_known ?switch - switch_device)
    (switch_exterior ?switch - switch_device)
    (switch_interior ?switch - switch_device)
    (stopper_state_known ?stopper - stopper_device)
    (stopper_open ?stopper - stopper_device)
    (stopper_closed ?stopper - stopper_device)
    (switches_ready ?side - rail_side)
    (stoppers_open ?side - rail_side)
    (switches_ready_for ?s - shuttle)
    (stoppers_open_for ?s - shuttle)
    (obstacle_present ?obs - obstacle ?side - rail_side)
    (waiting_for_clearance ?s - shuttle)
    (front_of ?front - shuttle ?rear - shuttle)
    (behind ?rear - shuttle ?front - shuttle)
    (goal_candidate ?s - shuttle)
    (station_only_goal)
    (target_slot_for_goal ?slot - slot)
    (target_station_for_goal ?station - station)
    (task_assigned ?s - shuttle ?station - station)
    (task_done ?s - shuttle ?station - station)
    (transport_goal_done ?station - station)
    (goal_slot_reached ?slot - slot)
    (inspection_required ?target - inspection_target)
    (inspection_done ?target - inspection_target)
  )

  (:functions
    (total-cost)
    (route_cost ?from - slot ?to - slot)
    (pending_clearances ?side - rail_side)
    (clearance_cursor ?side - rail_side)
    (clearance_order ?blocker - shuttle)
  )

  (:action prepare_switches
    :parameters (
      ?side - rail_side
      ?from - station
      ?to - station
      ?switches - switch_group
    )
    :precondition (and
      (validated_state)
      (connected ?side ?from ?to)
      (normal_route ?side)
    )
    :effect (and
      (switches_ready ?side)
      (increase (total-cost) 1)
    )
  )

  (:action open_stoppers
    :parameters (
      ?side - rail_side
      ?from - station
      ?to - station
      ?stoppers - stopper_group
    )
    :precondition (and
      (validated_state)
      (connected ?side ?from ?to)
      (switches_ready ?side)
      (normal_route ?side)
    )
    :effect (and
      (stoppers_open ?side)
      (path_ready ?side ?from ?to)
      (increase (total-cost) 1)
    )
  )

  (:action move_shuttle_to_slot
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from - station
      ?to - station
      ?from_slot - slot
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?s ?side)
      (slot_on_side ?from_slot ?side)
      (slot_on_side ?to_slot ?side)
      (slot_at_station ?from_slot ?from)
      (slot_at_station ?to_slot ?to)
      (shuttle_at ?s ?from)
      (shuttle_at_slot ?s ?from_slot)
      (connected ?side ?from ?to)
      (path_ready ?side ?from ?to)
      (switches_ready ?side)
      (stoppers_open ?side)
      (normal_route ?side)
      (= (pending_clearances ?side) 0)
      (route_clear_between ?from_slot ?to_slot)
      (slot_free ?to_slot)
    )
    :effect (and
      (not (shuttle_at ?s ?from))
      (not (shuttle_stopped_at ?s ?from))
      (not (shuttle_at_slot ?s ?from_slot))
      (not (slot_occupied_by ?from_slot ?s))
      (slot_free ?from_slot)
      (not (slot_free ?to_slot))
      (slot_occupied_by ?to_slot ?s)
      (slot_reserved_by ?to_slot ?s)
      (route_reserved_by ?from_slot ?to_slot ?s)
      (shuttle_at_slot ?s ?to_slot)
      (shuttle_at ?s ?to)
      (increase (total-cost) 10)
    )
  )

  ; Compile a route from the shuttle's accepted visual segment/position to
  ; the requested slot.  The executive expands this symbolic action into the
  ; exact mixed switch configuration derived from the authoritative rail
  ; topology, followed by opening the stoppers.  Re-observation must confirm
  ; that physical configuration before the move action becomes applicable.
  (:action prepare_topology_route
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from_block - block
      ?to_slot - slot
      ?switches - switch_group
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?s ?side)
      (shuttle_at_topology_block ?s ?from_block)
      (block_on_side ?from_block ?side)
      (slot_on_side ?to_slot ?side)
      (topology_route_available ?s ?from_block ?to_slot)
      (topology_route_clear ?s ?from_block ?to_slot)
      (slot_free ?to_slot)
      (= (pending_clearances ?side) 0)
    )
    :effect (and
      (topology_route_configured ?s ?from_block ?to_slot)
      (switches_ready ?side)
      (stoppers_open ?side)
      (increase (total-cost) 2)
    )
  )

  ; Move a shuttle whose learned location is a valid topology segment but is
  ; not close enough to any exterior slot.  Final stopping remains guarded by
  ; the identity-bearing deterministic sensor of the requested slot.
  (:action move_shuttle_from_segment_to_slot
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from_block - block
      ?to - station
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?s ?side)
      (shuttle_at_topology_block ?s ?from_block)
      (block_on_side ?from_block ?side)
      (slot_on_side ?to_slot ?side)
      (slot_at_station ?to_slot ?to)
      (topology_route_available ?s ?from_block ?to_slot)
      (topology_route_clear ?s ?from_block ?to_slot)
      (topology_route_configured ?s ?from_block ?to_slot)
      (= (pending_clearances ?side) 0)
      (slot_free ?to_slot)
    )
    :effect (and
      (not (shuttle_at_topology_block ?s ?from_block))
      (not (slot_free ?to_slot))
      (slot_occupied_by ?to_slot ?s)
      (slot_reserved_by ?to_slot ?s)
      (shuttle_at_slot ?s ?to_slot)
      (shuttle_at ?s ?to)
      (shuttle_stopped_at ?s ?to)
      (increase (total-cost) 10)
    )
  )

  ; Enter once and keep the A3/A4 interior route held for the complete
  ; multi-blocker clearance phase.
  (:action begin_route_clearance
    :parameters (
      ?selected - shuttle
      ?side - rail_side
      ?from_slot - slot
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (normal_route ?side)
      (shuttle_on_side ?selected ?side)
      (goal_candidate ?selected)
      (shuttle_at_slot ?selected ?from_slot)
      (target_slot_for_goal ?to_slot)
      (> (pending_clearances ?side) 0)
    )
    :effect (and
      (not (normal_route ?side))
      (clearance_mode ?side)
      (not (switches_ready ?side))
      (not (stoppers_open ?side))
      (increase (total-cost) 1)
    )
  )

  ; One ordered PlanSys2-owned blocker relocation. The runtime expands this
  ; into a supervised shuttle command and independently verified stop. It
  ; deliberately does not restore any switch or stopper.
  (:action relocate_blocker_to_interior
    :parameters (
      ?blocker - shuttle
      ?selected - shuttle
      ?side - rail_side
      ?from_slot - slot
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?blocker ?side)
      (shuttle_on_side ?selected ?side)
      (shuttle_at_slot ?selected ?from_slot)
      (target_slot_for_goal ?to_slot)
      (clearance_mode ?side)
      (clearance_precedes ?blocker ?selected)
      (route_blocked_by ?from_slot ?to_slot ?blocker)
      (clearance_destination_ready ?blocker)
      (> (pending_clearances ?side) 0)
      (= (clearance_order ?blocker) (clearance_cursor ?side))
    )
    :effect (and
      (clearance_relocated ?blocker)
      (not (route_blocked_by ?from_slot ?to_slot ?blocker))
      (decrease (pending_clearances ?side) 1)
      (increase (clearance_cursor ?side) 1)
      (increase (total-cost) 4)
    )
  )

  ; Restore the normal exterior route exactly once, and only after every
  ; ordered blocker relocation has completed.
  (:action finish_route_clearance
    :parameters (
      ?selected - shuttle
      ?side - rail_side
      ?from_slot - slot
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (clearance_mode ?side)
      (shuttle_on_side ?selected ?side)
      (goal_candidate ?selected)
      (shuttle_at_slot ?selected ?from_slot)
      (target_slot_for_goal ?to_slot)
      (= (pending_clearances ?side) 0)
    )
    :effect (and
      (not (clearance_mode ?side))
      (normal_route ?side)
      (switches_ready ?side)
      (stoppers_open ?side)
      (route_clear_between ?from_slot ?to_slot)
      (slot_free ?to_slot)
      (increase (total-cost) 1)
    )
  )

  (:action stop_shuttle
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from - station
      ?to - station
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?s ?side)
      (shuttle_at ?s ?to)
      (path_ready ?side ?from ?to)
    )
    :effect (and
      (shuttle_stopped_at ?s ?to)
      (not (path_ready ?side ?from ?to))
      (increase (total-cost) 1)
    )
  )

  (:action finish_task
    :parameters (
      ?s - shuttle
      ?station - station
    )
    :precondition (and
      (validated_state)
      (station_only_goal)
      (target_station_for_goal ?station)
      (shuttle_at ?s ?station)
      (shuttle_stopped_at ?s ?station)
    )
    :effect (and
      (task_done ?s ?station)
      (increase (total-cost) 1)
    )
  )

  (:action finish_candidate_task
    :parameters (
      ?s - shuttle
      ?station - station
      ?slot - slot
    )
    :precondition (and
      (validated_state)
      (goal_candidate ?s)
      (target_station_for_goal ?station)
      (target_slot_for_goal ?slot)
      (shuttle_at ?s ?station)
      (shuttle_stopped_at ?s ?station)
      (shuttle_at_slot ?s ?slot)
    )
    :effect (and
      (task_done ?s ?station)
      (transport_goal_done ?station)
      (goal_slot_reached ?slot)
      (increase (total-cost) 1)
    )
  )
)
