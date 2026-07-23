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


def _room315_smoke_trusted_status() -> dict[str, Any]:
    segment_by_slot = {'1': 'A1E', '2': 'A12E', '3': 'A23', '4': 'A34E'}
    sensor_by_side = {
        'right': {'1': 'DZI1R', '2': 'DZI2R', '3': 'DZI3R', '4': 'DZI4R'},
        'left': {'1': 'DZI1L', '2': 'DZI2L', '3': 'DZI3L', '4': 'DZI4L'},
    }
    rails: dict[str, Any] = {}
    for side in ('right', 'left'):
        active = [
            {
                'name': sensor_by_side[side][slot],
                'shuttle': f'room315_{side}_shuttle_{slot}',
                'segment': segment_by_slot[slot],
            }
            for slot in ('1', '2', '3', '4')
        ]
        rails[side] = {
            'switches': {f'A{index}': 'EXTERIOR' for index in range(1, 5)},
            'stoppers': {f'A{index}': 'open' for index in range(1, 5)},
            'payloads': {
                f'room315_{side}_shuttle_{index}': {
                    'loaded': side == 'right' and index == 1,
                }
                for index in range(1, 5)
            },
            'active_position_sensors': active,
            'obstacles': {},
        }
    return {
        'timestamp': 10.0,
        'rails': rails,
        'obstacles': {'right': [], 'left': []},
    }


def _room315_smoke_visual_label() -> dict[str, Any]:
    return {
        'visual_state_labels': {
            'schema_version': VISUAL_STATE_SCHEMA_VERSION,
            'calibration_version': 'room315_visual_observed_state_v1',
            'scenario_family': 'visual_state_plansys2_smoke',
            'confidence': 0.96,
            'shuttles': [
                {
                    'id': 'R1',
                    'visually_available_identity': 'R1',
                    'bbox': [55.0, 45.0, 10.0, 10.0],
                    'location': {'side': 'right', 'slot': '1'},
                    'loaded_state': 'loaded',
                    'confidence': 0.94,
                }
            ],
            'switches': [
                {'id': 'right:A1', 'state': 'EXTERIOR', 'confidence': 0.91},
            ],
            'obstacles': [],
        }
    }


def visual_state_plansys2_smoke() -> dict[str, Any]:
    provider = load_local_script_module('room_315_visual_observed_state_provider')
    generator = load_local_script_module('room_315_pddl_scenario_generator')
    scene = visual_label_to_provider_compact_scene(_room315_smoke_visual_label(), timestamp=10.0)
    adapter = provider.StrictJsonCompactModelAdapter()
    adapter.parse(scene)
    bad_scene = copy.deepcopy(scene)
    bad_scene['detections'][0]['rail_command'] = {'action': 'shuttle', 'command': 'ON'}
    command_boundary_rejected = False
    try:
        adapter.parse(bad_scene)
    except provider.VisualObservationError:
        command_boundary_rejected = True

    trusted_status = _room315_smoke_trusted_status()
    visual_state = provider.VisualObservedStateProvider(
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
