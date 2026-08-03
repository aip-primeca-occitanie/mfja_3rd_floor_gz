; Room 315 high-level shuttle planning domain.
;
; Production problems are built from validated ObservedState + TaskGoal
; contracts. Unknown, stale, or conflicting state is handled before this domain
; is called, so absence of a predicate here is never used as a perception
; default. The symbolic actions below map either to a real supervisor primitive
; or to a documented deterministic macro in room_315_pddl_plan_translator.py.

(define (domain room315-shuttle)
  (:requirements :strips :typing :negative-preconditions :fluents)

  (:types
    inspectable - object
    inspection_target rail_side shuttle station slot - inspectable
    block switch_device stopper_device switch_group stopper_group obstacle - object
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
    (segment_only_location ?s - shuttle)
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
    (interior_entry_route_clear ?blocker - shuttle)
    (normal_route ?side - rail_side)
    (clearance_mode ?side - rail_side)
    (clearance_pause_safe ?side - rail_side)
    (route_reconfiguration_required ?side - rail_side)
    (route_reconfiguration_safe ?side - rail_side)
    (route_reserved_by ?from - slot ?to - slot ?s - shuttle)

    (switch_state_known ?switch - switch_device)
    (switch_exterior ?switch - switch_device)
    (switch_interior ?switch - switch_device)
    (stopper_state_known ?stopper - stopper_device)
    (stopper_open ?stopper - stopper_device)
    (stopper_closed ?stopper - stopper_device)
    (switches_ready ?side - rail_side)
    (stoppers_open ?side - rail_side)
    (switch_group_on_side ?group - switch_group ?side - rail_side)
    (stopper_group_on_side ?group - stopper_group ?side - rail_side)
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

    (inspection_required ?target - inspectable)
    (inspection_done ?target - inspectable)
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
      (switch_group_on_side ?switches ?side)
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
      (stopper_group_on_side ?stoppers ?side)
      (switches_ready ?side)
      (normal_route ?side)
    )
    :effect (and
      (stoppers_open ?side)
      (path_ready ?side ?from ?to)
      (increase (total-cost) 1)
    )
  )

  (:action restore_normal_route
    :parameters (
      ?side - rail_side
      ?from - station
      ?to - station
    )
    :precondition (and
      (validated_state)
      (connected ?side ?from ?to)
      (route_reconfiguration_required ?side)
      (route_reconfiguration_safe ?side)
      (not (clearance_mode ?side))
      (= (pending_clearances ?side) 0)
    )
    :effect (and
      (not (route_reconfiguration_required ?side))
      (not (route_reconfiguration_safe ?side))
      (not (clearance_mode ?side))
      (normal_route ?side)
      (switches_ready ?side)
      (stoppers_open ?side)
      (path_ready ?side ?from ?to)
      (increase (total-cost) 2)
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
      (slot_occupied_by ?from_slot ?s)
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
      (segment_only_location ?s)
      (shuttle_at_topology_block ?s ?from_block)
      (block_on_side ?from_block ?side)
      (slot_on_side ?to_slot ?side)
      (topology_route_available ?s ?from_block ?to_slot)
      (topology_route_clear ?s ?from_block ?to_slot)
      (switch_group_on_side ?switches ?side)
      (slot_free ?to_slot)
      (= (pending_clearances ?side) 0)
      (normal_route ?side)
    )
    :effect (and
      (topology_route_configured ?s ?from_block ?to_slot)
      (switches_ready ?side)
      (stoppers_open ?side)
      (increase (total-cost) 2)
    )
  )

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
      (segment_only_location ?s)
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

  ; Configure an authoritative alternate route for a shuttle that is still
  ; represented at an exact source slot. The source topology block preserves
  ; the visual branch/segment needed by the executive route certificate.
  (:action prepare_slot_topology_route
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from_slot - slot
      ?from_block - block
      ?to_slot - slot
      ?switches - switch_group
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?s ?side)
      (slot_on_side ?from_slot ?side)
      (slot_on_side ?to_slot ?side)
      (shuttle_at_slot ?s ?from_slot)
      (slot_occupied_by ?from_slot ?s)
      (shuttle_at_topology_block ?s ?from_block)
      (block_on_side ?from_block ?side)
      (topology_route_available ?s ?from_block ?to_slot)
      (topology_route_clear ?s ?from_block ?to_slot)
      (switch_group_on_side ?switches ?side)
      (slot_free ?to_slot)
      (= (pending_clearances ?side) 0)
      (normal_route ?side)
    )
    :effect (and
      (topology_route_configured ?s ?from_block ?to_slot)
      (switches_ready ?side)
      (stoppers_open ?side)
      (increase (total-cost) 2)
    )
  )

  ; Execute the configured alternate topology route and atomically transfer
  ; symbolic occupancy from the known source slot to the requested target.
  (:action move_shuttle_via_topology_to_slot
    :parameters (
      ?s - shuttle
      ?side - rail_side
      ?from_slot - slot
      ?from_block - block
      ?from - station
      ?to - station
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?s ?side)
      (slot_on_side ?from_slot ?side)
      (slot_on_side ?to_slot ?side)
      (slot_at_station ?from_slot ?from)
      (slot_at_station ?to_slot ?to)
      (shuttle_at_slot ?s ?from_slot)
      (slot_occupied_by ?from_slot ?s)
      (shuttle_at ?s ?from)
      (shuttle_at_topology_block ?s ?from_block)
      (block_on_side ?from_block ?side)
      (topology_route_available ?s ?from_block ?to_slot)
      (topology_route_clear ?s ?from_block ?to_slot)
      (topology_route_configured ?s ?from_block ?to_slot)
      (= (pending_clearances ?side) 0)
      (slot_free ?to_slot)
    )
    :effect (and
      (not (shuttle_at_topology_block ?s ?from_block))
      (not (topology_route_configured ?s ?from_block ?to_slot))
      (not (shuttle_at_slot ?s ?from_slot))
      (not (slot_occupied_by ?from_slot ?s))
      (not (shuttle_at ?s ?from))
      (not (shuttle_stopped_at ?s ?from))
      (slot_free ?from_slot)
      (not (slot_free ?to_slot))
      (slot_occupied_by ?to_slot ?s)
      (slot_reserved_by ?to_slot ?s)
      (route_reserved_by ?from_slot ?to_slot ?s)
      (shuttle_at_slot ?s ?to_slot)
      (shuttle_at ?s ?to)
      (shuttle_stopped_at ?s ?to)
      (increase (total-cost) 10)
    )
  )

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

  ; Runtime clearance macro: every physical sub-command remains supervised,
  ; ordered, and re-observed without restoring the switches between blockers.
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
      ; clearance_precedes is emitted only for the one frozen, proved next
      ; mover.  That mover may be the direct route blocker or a dependency
      ; that must create capacity before the direct blocker can move.
      (clearance_destination_ready ?blocker)
      (interior_entry_route_clear ?blocker)
      (> (pending_clearances ?side) 0)
      (= (clearance_order ?blocker) (clearance_cursor ?side))
    )
    :effect (and
      (clearance_relocated ?blocker)
      ; Route occupancy is rebuilt from the next accepted visual observation.
      ; Do not delete a direct-blocker atom here: a valid dependency mover may
      ; not own one, and POPF then prunes the otherwise applicable action.
      (decrease (pending_clearances ?side) 1)
      (increase (clearance_cursor ?side) 1)
      (increase (total-cost) 4)
    )
  )

  ; Topology-selected vacancy seed: stage the user's shuttle itself only when
  ; it is the unique safe first mover before A3.
  (:action stage_selected_to_interior
    :parameters (
      ?selected - shuttle
      ?side - rail_side
      ?from_slot - slot
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?selected ?side)
      (goal_candidate ?selected)
      (shuttle_at_slot ?selected ?from_slot)
      (target_slot_for_goal ?to_slot)
      (clearance_mode ?side)
      (clearance_destination_ready ?selected)
      (interior_entry_route_clear ?selected)
      (> (pending_clearances ?side) 0)
      (= (clearance_order ?selected) (clearance_cursor ?side))
    )
    :effect (and
      (clearance_relocated ?selected)
      (decrease (pending_clearances ?side) 1)
      (increase (clearance_cursor ?side) 1)
      (increase (total-cost) 4)
    )
  )

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
      (clearance_pause_safe ?side)
    )
    :effect (and
      (not (clearance_mode ?side))
      (normal_route ?side)
      (switches_ready ?side)
      (stoppers_open ?side)
      (route_clear_between ?from_slot ?to_slot)
      (increase (total-cost) 1)
    )
  )

  ; Segment-origin counterpart of begin_route_clearance.  The selected
  ; shuttle remains bound to its accepted visual topology block; no synthetic
  ; source-slot occupancy is introduced.
  (:action begin_segment_route_clearance
    :parameters (
      ?selected - shuttle
      ?side - rail_side
      ?from_block - block
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (normal_route ?side)
      (shuttle_on_side ?selected ?side)
      (goal_candidate ?selected)
      (segment_only_location ?selected)
      (shuttle_at_topology_block ?selected ?from_block)
      (block_on_side ?from_block ?side)
      (slot_on_side ?to_slot ?side)
      (target_slot_for_goal ?to_slot)
      (topology_route_available ?selected ?from_block ?to_slot)
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

  ; Relocate one blocker on the frozen topology route.  This action is kept
  ; separate from the exact-slot action because its route proof is expressed
  ; by topology_route_blocked_by rather than a fabricated source slot.
  (:action relocate_segment_blocker_to_interior
    :parameters (
      ?blocker - shuttle
      ?selected - shuttle
      ?side - rail_side
      ?from_block - block
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?blocker ?side)
      (shuttle_on_side ?selected ?side)
      (segment_only_location ?selected)
      (shuttle_at_topology_block ?selected ?from_block)
      (block_on_side ?from_block ?side)
      (slot_on_side ?to_slot ?side)
      (target_slot_for_goal ?to_slot)
      (clearance_mode ?side)
      (clearance_precedes ?blocker ?selected)
      ; clearance_precedes authorizes either a direct topology blocker or the
      ; proved dependency mover selected by the capacity search.
      (clearance_destination_ready ?blocker)
      (interior_entry_route_clear ?blocker)
      (> (pending_clearances ?side) 0)
      (= (clearance_order ?blocker) (clearance_cursor ?side))
    )
    :effect (and
      (clearance_relocated ?blocker)
      ; The receding-horizon rebuild owns route-effect recomputation.  A
      ; capacity dependency is authorized by clearance_precedes but is not a
      ; fabricated direct topology blocker, so no blocker atom is deleted.
      (decrease (pending_clearances ?side) 1)
      (increase (clearance_cursor ?side) 1)
      (increase (total-cost) 4)
    )
  )

  (:action stage_selected_segment_to_interior
    :parameters (
      ?selected - shuttle
      ?side - rail_side
      ?from_block - block
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (shuttle_on_side ?selected ?side)
      (goal_candidate ?selected)
      (segment_only_location ?selected)
      (shuttle_at_topology_block ?selected ?from_block)
      (block_on_side ?from_block ?side)
      (target_slot_for_goal ?to_slot)
      (clearance_mode ?side)
      (clearance_destination_ready ?selected)
      (interior_entry_route_clear ?selected)
      (> (pending_clearances ?side) 0)
      (= (clearance_order ?selected) (clearance_cursor ?side))
    )
    :effect (and
      (clearance_relocated ?selected)
      (decrease (pending_clearances ?side) 1)
      (increase (clearance_cursor ?side) 1)
      (increase (total-cost) 4)
    )
  )

  ; Restore the normal route for a segment-origin goal only after the ordered
  ; relocation certificate sequence is complete.  This establishes the
  ; topology-route fact consumed by prepare_topology_route.
  (:action finish_segment_route_clearance
    :parameters (
      ?selected - shuttle
      ?side - rail_side
      ?from_block - block
      ?to_slot - slot
    )
    :precondition (and
      (validated_state)
      (clearance_mode ?side)
      (shuttle_on_side ?selected ?side)
      (goal_candidate ?selected)
      (segment_only_location ?selected)
      (shuttle_at_topology_block ?selected ?from_block)
      (block_on_side ?from_block ?side)
      (slot_on_side ?to_slot ?side)
      (target_slot_for_goal ?to_slot)
      (topology_route_available ?selected ?from_block ?to_slot)
      (= (pending_clearances ?side) 0)
      (clearance_pause_safe ?side)
    )
    :effect (and
      (not (clearance_mode ?side))
      (normal_route ?side)
      (switches_ready ?side)
      (stoppers_open ?side)
      (topology_route_clear ?selected ?from_block ?to_slot)
      (increase (total-cost) 1)
    )
  )

  ; The A34I buffer can hold only physically separated stopped shuttles.  When
  ; that certified capacity is exhausted, restore the exterior route as one
  ; explicit receding-horizon step.  A fresh problem then performs safe
  ; exterior-slot choreography before any further clearance phase.
  (:action pause_route_clearance
    :parameters (?side - rail_side)
    :precondition (and
      (validated_state)
      (clearance_mode ?side)
      (clearance_pause_safe ?side)
      (> (pending_clearances ?side) 0)
    )
    :effect (and
      (not (clearance_mode ?side))
      (normal_route ?side)
      (switches_ready ?side)
      (stoppers_open ?side)
      (increase (total-cost) 2)
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

  (:action inspect_state
    :parameters (
      ?target - inspectable
    )
    :precondition (and
      (validated_state)
      (inspection_required ?target)
    )
    :effect (and
      (inspection_done ?target)
      (increase (total-cost) 1)
    )
  )
)
