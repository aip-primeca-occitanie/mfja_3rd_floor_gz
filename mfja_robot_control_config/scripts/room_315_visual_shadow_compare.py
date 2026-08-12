#!/usr/bin/env python3
"""Observation-only paired-frame comparison for V3 active and V4 shadow."""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from mfja_rail_interfaces.msg import VisualStateObservation
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from room_315_runtime_acceptance_recorder import observation_dict


def _pair_key(observation: dict[str, Any]) -> tuple[int, int]:
    return (
        round(float(observation.get('left_image_stamp_s', -1.0)) * 1e9),
        round(float(observation.get('right_image_stamp_s', -1.0)) * 1e9),
    )


def compare_paired_observations(
    v3: dict[str, Any],
    v4: dict[str, Any],
) -> dict[str, Any]:
    """Compare structured outputs produced from the exact same image pair."""

    if _pair_key(v3) != _pair_key(v4):
        raise ValueError('shadow observations do not share the same image stamps')
    by_generation: dict[str, dict[str, dict[str, Any]]] = {}
    for generation, payload in (('v3', v3), ('v4', v4)):
        rows = payload.get('shuttles')
        if not isinstance(rows, list):
            raise ValueError(f'{generation} observation shuttles must be a list')
        parsed: dict[str, dict[str, Any]] = {}
        for row in rows:
            if not isinstance(row, dict):
                raise ValueError(f'{generation} observation has invalid shuttle row')
            identity = str(row.get('identity') or '').strip().upper()
            if not identity or identity in parsed:
                raise ValueError(
                    f'{generation} observation has invalid/duplicate identity'
                )
            parsed[identity] = row
        by_generation[generation] = parsed
    if set(by_generation['v3']) != set(by_generation['v4']):
        raise ValueError('shadow observations have different fixed identity sets')

    present = {
        identity
        for identity, row in by_generation['v4'].items()
        if row.get('presence_state') == 'present'
    }
    per_identity: dict[str, Any] = {}
    segment_agreements = 0
    loaded_agreements = 0
    ratio_differences: list[float] = []
    wrong_v4_sides: list[str] = []
    for identity in sorted(present):
        old = by_generation['v3'][identity]
        new = by_generation['v4'][identity]
        segment_equal = str(old.get('block')) == str(new.get('block'))
        loaded_equal = str(old.get('loaded_state')) == str(new.get('loaded_state'))
        ratio_difference = abs(
            float(old.get('s_ratio', 0.0)) - float(new.get('s_ratio', 0.0))
        )
        expected_side = 'left' if identity.startswith('L') else 'right'
        if str(new.get('side')) != expected_side:
            wrong_v4_sides.append(identity)
        segment_agreements += int(segment_equal)
        loaded_agreements += int(loaded_equal)
        ratio_differences.append(ratio_difference)
        per_identity[identity] = {
            'segment_agreement': segment_equal,
            'loaded_agreement': loaded_equal,
            'absolute_s_ratio_difference': ratio_difference,
            'v3_segment': old.get('block'),
            'v4_segment': new.get('block'),
        }
    count = len(present)
    return {
        'pair_key_ns': list(_pair_key(v4)),
        'present_identity_count': count,
        'v3_accepted': bool(v3.get('accepted')),
        'v4_accepted': bool(v4.get('accepted')),
        'segment_agreement_count': segment_agreements,
        'loaded_agreement_count': loaded_agreements,
        'maximum_absolute_s_ratio_difference': max(ratio_differences, default=0.0),
        'wrong_v4_side_identities': wrong_v4_sides,
        'per_identity': per_identity,
        'quality_ground_truth_available': False,
    }


@dataclass
class ShadowComparisonAccumulator:
    expected_v3_checkpoint_sha256: str
    expected_v4_checkpoint_sha256: str
    minimum_paired_frames: int = 10
    pending: dict[tuple[int, int], dict[str, dict[str, Any]]] = field(
        default_factory=dict
    )
    comparisons: list[dict[str, Any]] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def add(self, generation: str, observation: dict[str, Any]) -> None:
        if generation not in {'v3', 'v4'}:
            raise ValueError('generation must be v3 or v4')
        expected = (
            self.expected_v3_checkpoint_sha256
            if generation == 'v3'
            else self.expected_v4_checkpoint_sha256
        )
        if str(observation.get('checkpoint_sha256') or '') != expected:
            self.errors.append(f'{generation}_checkpoint_sha256_mismatch')
            return
        key = _pair_key(observation)
        if min(key) < 0:
            self.errors.append(f'{generation}_invalid_image_stamps')
            return
        row = self.pending.setdefault(key, {})
        if generation in row:
            self.errors.append(f'{generation}_duplicate_frame:{key}')
            return
        row[generation] = observation
        if set(row) == {'v3', 'v4'}:
            try:
                self.comparisons.append(
                    compare_paired_observations(row['v3'], row['v4'])
                )
            except (TypeError, ValueError) as exc:
                self.errors.append(f'comparison_failed:{exc}')
            del self.pending[key]
        if len(self.pending) > 200:
            oldest = sorted(self.pending)[:-200]
            for stale in oldest:
                del self.pending[stale]

    def report(self) -> dict[str, Any]:
        pair_count = len(self.comparisons)
        present_count = sum(
            int(row['present_identity_count']) for row in self.comparisons
        )
        v3_accepted = sum(bool(row['v3_accepted']) for row in self.comparisons)
        v4_accepted = sum(bool(row['v4_accepted']) for row in self.comparisons)
        segment_agreement = sum(
            int(row['segment_agreement_count']) for row in self.comparisons
        )
        loaded_agreement = sum(
            int(row['loaded_agreement_count']) for row in self.comparisons
        )
        wrong_sides = sorted({
            identity
            for row in self.comparisons
            for identity in row['wrong_v4_side_identities']
        })
        acceptance_coverage = v4_accepted / pair_count if pair_count else 0.0
        passed = bool(
            not self.errors
            and pair_count >= int(self.minimum_paired_frames)
            and acceptance_coverage >= 0.90
            and not wrong_sides
        )
        return {
            'schema_version': 'room315.visual_shadow_comparison.v4.v1',
            'status': 'passed' if passed else 'failed',
            'role': 'observation_only_same_frame_shadow',
            'automatic_runtime_switch': False,
            'control_isolation': {
                'dry_run_state_fusion': True,
                'plansys2_update_enabled': False,
                'plansys2_mutation_count': 0,
                'actuation_command_count': 0,
                'comparator_owns_command_publisher': False,
                'evidence_scope': (
                    'canonical_shadow_launch_contract_and_runtime_node_ownership'
                ),
            },
            'quality_ground_truth_available': False,
            'expected_checkpoint_sha256': {
                'v3': self.expected_v3_checkpoint_sha256,
                'v4': self.expected_v4_checkpoint_sha256,
            },
            'minimum_paired_frames': int(self.minimum_paired_frames),
            'paired_frame_count': pair_count,
            'present_slot_comparison_count': present_count,
            'v3_accepted_frame_count': v3_accepted,
            'v4_accepted_frame_count': v4_accepted,
            'v4_acceptance_coverage': acceptance_coverage,
            'segment_agreement_rate': (
                segment_agreement / present_count if present_count else 0.0
            ),
            'loaded_agreement_rate': (
                loaded_agreement / present_count if present_count else 0.0
            ),
            'maximum_absolute_s_ratio_difference': max(
                (
                    float(row['maximum_absolute_s_ratio_difference'])
                    for row in self.comparisons
                ),
                default=0.0,
            ),
            'wrong_v4_side_identities': wrong_sides,
            'unpaired_frame_count': len(self.pending),
            'errors': list(dict.fromkeys(self.errors)),
            'interpretation': (
                'Agreement is a migration diagnostic only; V3 is not ground '
                'truth. Scenario-grounded runtime acceptance is evaluated '
                'separately.'
            ),
        }


class Room315VisualShadowCompareNode(Node):
    def __init__(self) -> None:
        super().__init__('room_315_visual_shadow_compare')
        for name, default in {
            'v3_validation_topic': '/room_315/visual_state/validation',
            'v4_validation_topic': '/room_315/visual_state/shadow_v4/validation',
            'expected_v3_checkpoint_sha256': '',
            'expected_v4_checkpoint_sha256': '',
            'minimum_paired_frames': 10,
            'duration_s': 30.0,
            'output_file': '',
        }.items():
            self.declare_parameter(name, default)
        output = str(self.get_parameter('output_file').value).strip()
        if not output:
            raise RuntimeError('shadow comparison output_file is required')
        self.output_file = Path(output).expanduser().resolve()
        if self.output_file.exists():
            raise RuntimeError(f'refusing to overwrite shadow report: {self.output_file}')
        self.accumulator = ShadowComparisonAccumulator(
            expected_v3_checkpoint_sha256=self._string(
                'expected_v3_checkpoint_sha256'
            ),
            expected_v4_checkpoint_sha256=self._string(
                'expected_v4_checkpoint_sha256'
            ),
            minimum_paired_frames=int(
                self.get_parameter('minimum_paired_frames').value
            ),
        )
        if not all((
            self.accumulator.expected_v3_checkpoint_sha256,
            self.accumulator.expected_v4_checkpoint_sha256,
        )):
            raise RuntimeError('both expected shadow checkpoint hashes are required')
        self.started = time.monotonic()
        self.finished = False
        self.exit_code = 1
        self.create_subscription(
            VisualStateObservation,
            self._string('v3_validation_topic'),
            lambda message: self._on_observation('v3', message),
            10,
        )
        self.create_subscription(
            VisualStateObservation,
            self._string('v4_validation_topic'),
            lambda message: self._on_observation('v4', message),
            10,
        )
        self.create_timer(0.25, self._tick)

    def _string(self, name: str) -> str:
        return str(self.get_parameter(name).value).strip()

    def _on_observation(
        self,
        generation: str,
        message: VisualStateObservation,
    ) -> None:
        self.accumulator.add(generation, observation_dict(message))

    def _tick(self) -> None:
        if self.finished:
            return
        if time.monotonic() - self.started >= float(
            self.get_parameter('duration_s').value
        ):
            self.finalize()
            rclpy.shutdown()

    def finalize(self) -> None:
        if self.finished:
            return
        self.finished = True
        report = self.accumulator.report()
        self.exit_code = 0 if report['status'] == 'passed' else 1
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_file.with_name(
            f'.{self.output_file.name}.tmp-{os.getpid()}'
        )
        temporary.write_text(
            json.dumps(report, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
        os.replace(temporary, self.output_file)


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = Room315VisualShadowCompareNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finalize()
        exit_code = node.exit_code
        node.destroy_node()
        rclpy.try_shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
