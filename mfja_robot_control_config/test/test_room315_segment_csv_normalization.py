#!/usr/bin/env python3

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import pytest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_ROOT = REPO_ROOT / 'mfja_robot_control_config'
KINEMATICS_DIR = PACKAGE_ROOT / 'config' / 'room_315_kinematics'
RAW_SEGMENTS_DIR = KINEMATICS_DIR / 'raw_segments'
SCRIPT_PATH = PACKAGE_ROOT / 'scripts' / 'room_315_kinematic_shuttle.py'
SNAPSHOT_PATH = (
    Path(__file__).parent
    / 'fixtures'
    / 'room315_segment_csv_pre_normalization.json'
)


def _load_module():
    name = 'room_315_kinematic_shuttle_csv_normalization'
    spec = importlib.util.spec_from_file_location(name, SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _snapshot():
    return json.loads(SNAPSHOT_PATH.read_text(encoding='utf-8'))


def _network_config(side):
    path = KINEMATICS_DIR / f'rail_network_{side}.yaml'
    return path, yaml.safe_load(path.read_text(encoding='utf-8'))


def _preserved_topology_sha256(config):
    preserved = {
        'schema_version': config['schema_version'],
        'network_id': config['network_id'],
        'motion_policy': config['motion_policy'],
        'switch_state_space': config['switch_state_space'],
        'stopper_state_space': config['stopper_state_space'],
        'nodes': config['nodes'],
        'segments': {
            name: {
                key: value
                for key, value in segment.items()
                if key != 'csv'
            }
            for name, segment in config['segments'].items()
        },
        'switches': config['switches'],
        'fixed_transitions': config['fixed_transitions'],
        'routing_table': config['routing_table'],
    }
    payload = json.dumps(
        preserved,
        sort_keys=True,
        separators=(',', ':'),
    ).encode()
    return hashlib.sha256(payload).hexdigest()


def _topology_edges(config):
    edges = []
    states = config['switch_state_space']['values']
    for source, rule in config['routing_table'].items():
        if rule['type'] == 'fixed':
            edges.append([source, 'fixed', rule['next_segment']])
            continue
        for state in states:
            target = rule['by_state'].get(
                state,
                rule.get('on_unknown_state'),
            )
            edges.append([source, f"{rule['switch']}={state}", target])
        edges.append(
            [
                source,
                f"{rule['switch']}=UNKNOWN",
                rule['on_unknown_state'],
            ]
        )
    return edges


def _point_tuples(segment):
    return [(point.x, point.y, point.z) for point in segment.points]


def test_public_segments_have_unique_explicit_normalized_csv_references():
    snapshot = _snapshot()
    public_segments = set(snapshot['segments'])
    actual_csv_files = sorted(RAW_SEGMENTS_DIR.glob('*.csv'))

    assert snapshot['segment_count'] == 14
    assert snapshot['csv_count'] == 14
    assert len(actual_csv_files) == snapshot['csv_count']
    assert {path.stem for path in actual_csv_files} == public_segments
    assert not list(RAW_SEGMENTS_DIR.glob('.room315_csv_migration_*'))

    for side in ('right', 'left'):
        _, config = _network_config(side)
        assert config['csv_reference_schema'] == (
            'public_segment_filename_v1'
        )
        assert set(config['segments']) == public_segments
        references = [
            segment['csv']
            for segment in config['segments'].values()
        ]
        assert len(references) == len(set(references)) == 14
        for public_name, segment in config['segments'].items():
            assert segment['csv'] == f'raw_segments/{public_name}.csv'


def test_normalized_csv_bytes_and_coordinates_match_pre_migration_snapshot():
    snapshot = _snapshot()
    coordinate_count = 0

    for public_name, expected in snapshot['segments'].items():
        path = RAW_SEGMENTS_DIR / f'{public_name}.csv'
        payload = path.read_bytes()
        assert len(payload) == expected['size_bytes']
        assert hashlib.sha256(payload).hexdigest() == expected['sha256']

        with path.open(newline='') as handle:
            rows = list(csv.DictReader(handle))
        assert len(rows) == expected['row_count']
        assert len(rows) == expected['parsed_coordinate_count']
        coordinate_count += len(rows)
        assert [
            rows[0][field] for field in ('index', 'x', 'y', 'z')
        ] == expected['first_sample']
        assert [
            rows[-1][field] for field in ('index', 'x', 'y', 'z')
        ] == expected['last_sample']
    assert coordinate_count == snapshot['total_coordinate_count']


def test_nodes_switches_successors_and_falling_edges_are_unchanged():
    snapshot = _snapshot()

    for side in ('right', 'left'):
        _, config = _network_config(side)
        assert len(config['segments']) == 14
        assert len(config['nodes']) == 12
        assert _preserved_topology_sha256(config) == (
            snapshot['preserved_topology_sha256'][side]
        )
        assert _topology_edges(config) == snapshot['topology_edges'][side]

        for public_name, expected in snapshot['segments'].items():
            segment = config['segments'][public_name]
            assert segment['start_node'] == expected['start_node']
            assert segment['end_node'] == expected['end_node']
            assert segment['direction'] == expected['direction']
            assert config['routing_table'][public_name] == (
                expected[f'{side}_successor_rule']
            )


@pytest.mark.parametrize('side', ('right', 'left'))
@pytest.mark.parametrize('backend', ('polyline', 'cubic_hermite'))
def test_all_normalized_segments_load_for_both_rails(side, backend):
    module = _load_module()
    network_path, _ = _network_config(side)

    network = module.RailNetwork.from_yaml(
        network_path,
        path_backend=backend,
    )

    assert set(network.segments) == set(_snapshot()['segments'])
    assert all(segment.length > 0.0 for segment in network.segments.values())


@pytest.mark.parametrize('side', ('right', 'left'))
def test_exact_legacy_yaml_references_remain_read_compatible(
    side,
    tmp_path,
):
    module = _load_module()
    snapshot = _snapshot()
    _, normalized_config = _network_config(side)
    legacy_config = dict(normalized_config)
    legacy_config.pop('csv_reference_schema')
    legacy_config['segments'] = {
        name: {
            **segment,
            'csv': snapshot['segments'][name]['legacy_csv'],
        }
        for name, segment in normalized_config['segments'].items()
    }
    legacy_path = tmp_path / f'legacy_{side}.yaml'
    legacy_path.write_text(
        yaml.safe_dump(legacy_config, sort_keys=False),
        encoding='utf-8',
    )

    with pytest.warns(
        DeprecationWarning,
        match='Loaded legacy Room 315 segment CSV references',
    ):
        legacy_network = module.RailNetwork.from_yaml(legacy_path)
    normalized_network = module.RailNetwork.from_yaml(
        KINEMATICS_DIR / f'rail_network_{side}.yaml'
    )

    for name in snapshot['segments']:
        assert _point_tuples(legacy_network.segments[name]) == (
            _point_tuples(normalized_network.segments[name])
        )


def test_explicit_yaml_csv_field_remains_authoritative(tmp_path):
    module = _load_module()
    _, config = _network_config('right')
    config['segments'] = {
        name: {
            **segment,
            'csv': str(RAW_SEGMENTS_DIR / f'{name}.csv'),
        }
        for name, segment in config['segments'].items()
    }
    custom_csv = tmp_path / 'deliberately_non_matching_name.csv'
    custom_csv.write_bytes((RAW_SEGMENTS_DIR / 'A3E.csv').read_bytes())
    config['segments']['A23']['csv'] = str(custom_csv)
    custom_yaml = tmp_path / 'explicit_mapping.yaml'
    custom_yaml.write_text(
        yaml.safe_dump(config, sort_keys=False),
        encoding='utf-8',
    )

    custom_network = module.RailNetwork.from_yaml(custom_yaml)
    canonical_network = module.RailNetwork.from_yaml(
        KINEMATICS_DIR / 'rail_network_right.yaml'
    )

    assert _point_tuples(custom_network.segments['A23']) == (
        _point_tuples(canonical_network.segments['A3E'])
    )
    assert _point_tuples(custom_network.segments['A23']) != (
        _point_tuples(canonical_network.segments['A23'])
    )
