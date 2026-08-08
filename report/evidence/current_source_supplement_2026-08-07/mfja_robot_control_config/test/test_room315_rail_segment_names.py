#!/usr/bin/env python3

import sys
from pathlib import Path

import pytest
import yaml


SCRIPT_DIR = Path(__file__).resolve().parents[1] / 'scripts'
KINEMATICS_DIR = (
    Path(__file__).resolve().parents[1] / 'config' / 'room_315_kinematics'
)
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from room_315_rail_defaults import LEFT_INTERNAL_TO_PUBLIC_SEGMENT_NAME_MAP
from room_315_rail_defaults import LEFT_PUBLIC_SEGMENT_NAME_MAP
from room_315_rail_defaults import LEFT_PUBLIC_TO_INTERNAL_SEGMENT_NAME_MAP
from room_315_rail_defaults import internal_rail_segment_name_to_public
from room_315_rail_defaults import public_rail_segment_name_to_internal


LEFT_INTERNAL_SEGMENTS = (
    'A1E', 'A1I', 'A2E', 'A2I', 'A3E', 'A3I', 'A4E', 'A4I',
    'A12E', 'A12I', 'A14', 'A23', 'A34E', 'A34I',
)


def test_left_segment_name_maps_are_explicit_inverses():
    assert (
        LEFT_PUBLIC_TO_INTERNAL_SEGMENT_NAME_MAP
        is not LEFT_INTERNAL_TO_PUBLIC_SEGMENT_NAME_MAP
    )
    assert set(LEFT_INTERNAL_TO_PUBLIC_SEGMENT_NAME_MAP) == set(
        LEFT_INTERNAL_SEGMENTS
    )
    assert LEFT_PUBLIC_TO_INTERNAL_SEGMENT_NAME_MAP == {
        public_name: internal_name
        for internal_name, public_name in
        LEFT_INTERNAL_TO_PUBLIC_SEGMENT_NAME_MAP.items()
    }


@pytest.mark.parametrize('internal_segment', LEFT_INTERNAL_SEGMENTS)
def test_left_segment_name_conversion_round_trip(internal_segment):
    public_segment = internal_rail_segment_name_to_public(
        'left',
        internal_segment,
    )
    assert public_rail_segment_name_to_internal(
        'left',
        public_segment,
    ) == internal_segment


def test_public_left_a23_is_internal_a14_in_both_conversion_directions():
    assert internal_rail_segment_name_to_public('left', 'A14') == 'A23'
    assert public_rail_segment_name_to_internal('left', 'A23') == 'A14'


def test_right_segment_names_are_identity_conversions():
    assert internal_rail_segment_name_to_public('right', ' a23 ') == 'A23'
    assert public_rail_segment_name_to_internal('right', ' a23 ') == 'A23'


def test_legacy_map_remains_an_internal_to_public_alias():
    assert LEFT_PUBLIC_SEGMENT_NAME_MAP is LEFT_INTERNAL_TO_PUBLIC_SEGMENT_NAME_MAP


@pytest.mark.parametrize('segment', LEFT_INTERNAL_SEGMENTS)
def test_directional_helpers_preserve_legacy_left_runtime_values(segment):
    legacy_value = LEFT_PUBLIC_SEGMENT_NAME_MAP.get(segment, segment)

    assert internal_rail_segment_name_to_public('left', segment) == legacy_value
    assert public_rail_segment_name_to_internal('left', segment) == legacy_value


def _public_routing_table(side):
    config = yaml.safe_load(
        (KINEMATICS_DIR / f'rail_network_{side}.yaml').read_text(
            encoding='utf-8',
        )
    )
    result = {}
    for internal_segment, internal_rule in config['routing_table'].items():
        public_rule = dict(internal_rule)
        if 'next_segment' in public_rule:
            public_rule['next_segment'] = internal_rail_segment_name_to_public(
                side,
                public_rule['next_segment'],
            )
        if 'by_state' in public_rule:
            public_rule['by_state'] = {
                state: (
                    next_segment
                    if next_segment == 'FALLING'
                    else internal_rail_segment_name_to_public(side, next_segment)
                )
                for state, next_segment in public_rule['by_state'].items()
            }
        result[internal_rail_segment_name_to_public(side, internal_segment)] = (
            public_rule
        )
    return result


def test_mirrored_rails_have_the_same_public_successor_contract():
    right = _public_routing_table('right')
    left = _public_routing_table('left')

    assert left == right
    assert left['A23']['switch'] == 'A3'
    assert left['A23']['by_state'] == {'E': 'A3E', 'I': 'A3I'}
