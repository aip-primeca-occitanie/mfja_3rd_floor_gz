#!/usr/bin/env python3
"""Visual-state to PlanSys2 integration smoke helpers.

This module owns deterministic fixtures and boundary checks. Training and
evaluation import the public helpers without carrying integration-test setup in
the model pipeline.
"""

import copy
import importlib.util
import sys
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_visual_state_dataset import VISUAL_STATE_SCHEMA_VERSION
from room_315_visual_state_dataset import normalize_visual_state_labels


def load_local_script_module(name: str) -> Any:
    if name in sys.modules:
        return sys.modules[name]
    path = SCRIPT_DIR / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f'cannot load {name} from {path}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _side_from_visual_identity(identity: str, fallback: str = 'right') -> str:
    text = str(identity or '').strip().casefold()
    if text.startswith('l') or 'left' in text:
        return 'left'
    if text.startswith('r') or 'right' in text:
        return 'right'
    return fallback


def visual_label_to_provider_compact_scene(
    label: dict[str, Any],
    *,
    timestamp: float = 0.0,
) -> dict[str, Any]:
    normalized = normalize_visual_state_labels(label)
    detections: list[dict[str, Any]] = []
    switches: list[dict[str, Any]] = []
    obstacles: list[dict[str, Any]] = []
    for shuttle in normalized.get('shuttles') or []:
        if not (
            shuttle.get('presence')
            and shuttle.get('visually_available')
        ):
            continue
        identity = str(
            shuttle.get('visually_available_identity')
            or shuttle.get('id')
            or 'unknown'
        )
        location = shuttle.get('location') if isinstance(shuttle.get('location'), dict) else {}
        side = str(location.get('side') or _side_from_visual_identity(identity))
        camera = f'overhead_{side}_rgbd'
        confidence = float(shuttle.get('confidence') or normalized.get('confidence') or 0.0)
        detections.append({
            'kind': 'shuttle',
            'id': str(shuttle.get('id') or identity),
            'camera': camera,
            'bbox': [float(value) for value in shuttle.get('bbox')],
            'identity': identity,
            'identity_confidence': confidence,
            'side': side,
            'loaded_state': str(shuttle.get('loaded_state') or 'unknown'),
            'loaded_confidence': confidence,
            'confidence': confidence,
            'timestamp': timestamp,
        })
    for switch in normalized.get('switches') or []:
        raw_id = str(switch.get('id') or '')
        side = 'left' if raw_id.casefold().startswith('left') else 'right'
        name = raw_id.split(':')[-1] if ':' in raw_id else raw_id
        switches.append({
            'id': raw_id or f'{side}:A1',
            'camera': f'overhead_{side}_rgbd',
            'bbox': [55.0, 45.0, 10.0, 10.0],
            'side': side,
            'name': str(name or 'A1').upper(),
            'state': str(switch.get('state') or 'unknown').upper(),
            'confidence': float(switch.get('confidence') or normalized.get('confidence') or 0.0),
            'timestamp': timestamp,
        })
    for obstacle in normalized.get('obstacles') or []:
        location = obstacle.get('location') if isinstance(obstacle.get('location'), dict) else {}
        side = str(location.get('side') or 'right')
        obstacles.append({
            'id': str(obstacle.get('id') or 'obstacle'),
            'camera': f'overhead_{side}_rgbd',
            'bbox': [float(value) for value in obstacle.get('bbox')],
            'side': side,
            'label': str(obstacle.get('id') or 'obstacle'),
            'confidence': float(obstacle.get('confidence') or normalized.get('confidence') or 0.0),
            'timestamp': timestamp,
        })
    return {
        'schema_version': 1,
        'timestamp': timestamp,
        'calibration_version': str(normalized.get('calibration_version') or ''),
        'detections': detections,
        'switches': switches,
        'obstacles': obstacles,
    }


def _room315_smoke_camera_info(width: int = 240, height: int = 100) -> dict[str, Any]:
    return {
        'width': width,
        'height': height,
        'k': [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0],
    }


def _room315_smoke_rgbd_streams() -> dict[str, Any]:
    width = 240
    height = 100
    depth = [[2.0 for _ in range(width)] for _ in range(height)]
    rgb = [[[0, 0, 0] for _ in range(width)] for _ in range(height)]
    return {
        'overhead_right_rgbd': {
            'rgb': rgb,
            'depth': depth,
            'camera_info': {
                **_room315_smoke_camera_info(width=width, height=height),
                'frame_id': 'overhead_right_rgbd_optical_frame',
            },
            'timestamp': 10.0,
        },
        'overhead_left_rgbd': {
            'rgb': rgb,
            'depth': depth,
            'camera_info': {
                **_room315_smoke_camera_info(width=width, height=height),
                'frame_id': 'overhead_left_rgbd_optical_frame',
            },
            'timestamp': 10.0,
        },
    }


def _room315_smoke_fixture(
    generator: Any,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Return a full-fleet visual fixture bound to the current slot topology.

    The planner deliberately rejects an exact DZI slot anchor unless the raw
    visual segment and longitudinal ratio agree.  Build the synthetic camera
    geometry from the same loaded topology so this smoke exercises that safety
    contract instead of carrying a second, stale slot-to-segment table.
    """

    topologies = {
        side: generator._planning_rail_topology(side)
        for side in ('right', 'left')
    }
    calibration: dict[str, Any] = {
        'schema_version': 1,
        'calibration_id': 'room315_visual_plansys2_smoke_v2',
        'thresholds': {
            'stale_after_s': 1.0,
            'min_detection_confidence': 0.6,
            'min_identity_confidence': 0.65,
            'min_loaded_confidence': 0.65,
            'rail_position_uncertainty_m': 0.05,
        },
        'cameras': {},
        'rail_geometry': {},
    }
    label: dict[str, Any] = {
        'visual_state_labels': {
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'calibration_version': calibration['calibration_id'],
            'scenario_family': 'visual_state_plansys2_smoke',
            'confidence': 0.96,
            'shuttles': [],
            'switches': [
                {
                    'id': 'right:A1',
                    'state': 'EXTERIOR',
                    'confidence': 0.91,
                },
            ],
            'obstacles': [],
        },
    }
    trusted_status: dict[str, Any] = {
        'timestamp': 10.0,
        'rails': {},
        'obstacles': {'right': [], 'left': []},
    }

    for side in ('right', 'left'):
        topology = topologies[side]
        rail_y = 0.0 if side == 'right' else 1.0
        camera_name = f'overhead_{side}_rgbd'
        calibration['cameras'][camera_name] = {
            'role': 'overhead_rgbd',
            'rail_side': side,
            'frame_id': f'{camera_name}_optical_frame',
            'depth_scale': 1.0,
            'room_from_camera': [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, rail_y],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
        slot_segments = sorted({
            location.segment
            for location in topology.slots.values()
        })
        segment_x = {
            segment: 2.0 * index
            for index, segment in enumerate(slot_segments)
        }
        geometry = {'segments': {}, 'slots': {}}
        for segment, start_x in segment_x.items():
            geometry['segments'][segment] = {
                'start_m': [start_x, rail_y, 2.0],
                'end_m': [start_x + 1.0, rail_y, 2.0],
                'max_distance_m': 0.12,
            }

        active_sensors: list[dict[str, Any]] = []
        payloads: dict[str, Any] = {}
        for slot in ('1', '2', '3', '4'):
            location = topology.slots[slot]
            x_m = segment_x[location.segment] + location.s_ratio
            geometry['slots'][slot] = {
                'center_m': [x_m, rail_y, 2.0],
                'radius_m': 0.08,
            }
            short_id = f'{side[0].upper()}{slot}'
            entity = f'room315_{side}_shuttle_{slot}'
            # With fx=100, cx=50, and depth=2 m, u=50+50*x.
            bbox_center_u = 50.0 + 50.0 * x_m
            label['visual_state_labels']['shuttles'].append({
                'id': short_id,
                'presence': True,
                'visually_available': True,
                'bbox': [bbox_center_u - 5.0, 45.0, 10.0, 10.0],
                'location': {
                    'side': side,
                    'block': location.segment,
                },
                'rail_position': {
                    'available': True,
                    's_m': location.s_ratio,
                    's_ratio': location.s_ratio,
                    'segment_length_m': 1.0,
                    'position_uncertainty_m': 0.0,
                },
                'loaded_state': 'loaded' if short_id == 'R1' else 'empty',
                'confidence': 0.94,
            })
            active_sensors.append({
                'name': f'DZI{slot}{side[0].upper()}',
                'shuttle': entity,
                'segment': location.segment,
            })
            payloads[entity] = {'loaded': short_id == 'R1'}

        calibration['rail_geometry'][side] = geometry
        trusted_status['rails'][side] = {
            'switches': {
                f'A{index}': 'EXTERIOR'
                for index in range(1, 5)
            },
            'stoppers': {
                f'A{index}': 'open'
                for index in range(1, 5)
            },
            'payloads': payloads,
            'active_position_sensors': active_sensors,
            'obstacles': {},
        }

    return calibration, trusted_status, label


def visual_state_plansys2_smoke() -> dict[str, Any]:
    provider = load_local_script_module('room_315_visual_observed_state_provider')
    generator = load_local_script_module('room_315_pddl_scenario_generator')
    calibration, trusted_status, label = _room315_smoke_fixture(generator)
    scene = visual_label_to_provider_compact_scene(label, timestamp=10.0)
    adapter = provider.StrictJsonCompactModelAdapter()
    adapter.parse(scene)
    bad_scene = copy.deepcopy(scene)
    bad_scene['detections'][0]['rail_command'] = {'action': 'shuttle', 'command': 'ON'}
    command_boundary_rejected = False
    try:
        adapter.parse(bad_scene)
    except provider.VisualObservationError:
        command_boundary_rejected = True

    visual_state = provider.VisualObservedStateProvider(
        calibration=calibration,
        compact_model=provider.DeterministicFixtureCompactModel(scene),
        trusted_status_snapshot=trusted_status,
        stale_after_s=1.0,
    ).observe(
        rgbd_streams=_room315_smoke_rgbd_streams(),
        timestamp=10.0,
    )
    visual_state = generator.ObservedState.from_dict(visual_state.to_dict())
    oracle_spec = generator.scenario_spec_from_case(
        'right_loaded_r1_s1_to_slot3_no_blocker_speed008'
    )
    oracle_state = generator._observed_state_from_scenario_spec(oracle_spec)
    task_goal = generator.TaskGoal(
        goal_id='visual-state-smoke-transport-r1-slot3',
        description='transport visual R1 to right slot 3',
        source='planner',
        timestamp=10.0,
        confidence=1.0,
        constraints={
            'goal_type': 'transport',
            'side': 'right',
            'target_kind': 'slot',
            'target_slot': '3',
            'shuttle_selection': 'explicit',
            'target_shuttle': 'right_shuttle_1',
            'payload_required': True,
        },
    )
    visual_problem = generator.build_pddl_problem_from_observed_state_task_goal(
        visual_state,
        task_goal,
        problem_name='room315-visual-state-smoke',
    )
    oracle_problem = generator.build_pddl_problem_from_observed_state_task_goal(
        oracle_state,
        task_goal,
        problem_name='room315-oracle-state-smoke',
    )
    visual_sources = sorted({fact.source for fact in visual_state.visual_model_inputs})
    command_like_visual_facts = [
        fact.to_dict()
        for fact in visual_state.visual_model_inputs
        if any(token in fact.predicate.casefold() for token in ('command', 'action', 'primitive'))
    ]
    return {
        'smoke_test': 'oracle_vs_visual_state_to_plansys2',
        'visual_state_provider': 'VisualObservedStateProvider',
        'oracle_reference': 'scenario_spec_observed_state_fixture',
        'plansys2_problem_built': True,
        'visual_problem_name': visual_problem.problem_name,
        'oracle_problem_name': oracle_problem.problem_name,
        'visual_goal_text': visual_problem.goal_text,
        'oracle_goal_text': oracle_problem.goal_text,
        'oracle_visual_goal_match': visual_problem.goal_text == oracle_problem.goal_text,
        'visual_problem_uses_plansys2': visual_problem.provenance.get('planner') == 'PlanSys2',
        'direct_command_capability': False,
        'compact_command_payload_rejected': command_boundary_rejected,
        'visual_fact_sources': visual_sources,
        'command_like_visual_fact_count': len(command_like_visual_facts),
        'published_commands': [],
    }


__all__ = [
    'load_local_script_module',
    'visual_label_to_provider_compact_scene',
    'visual_state_plansys2_smoke',
]
