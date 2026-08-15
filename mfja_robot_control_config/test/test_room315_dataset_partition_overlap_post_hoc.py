import json
import sys
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT_DIR = REPO_ROOT / 'mfja_robot_control_config' / 'scripts'
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import room_315_dataset_partition_overlap_post_hoc as audit


def _write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        ''.join(json.dumps(row, sort_keys=True) + '\n' for row in rows),
        encoding='utf-8',
    )


def _partition(tmp_path, name, *, s_ratio, shared_left=b'shared-left'):
    root = tmp_path / name
    dataset = root / 'dataset'
    episode = f'{name}_episode'
    left = dataset / 'episodes' / episode / 'left.jpg'
    right = dataset / 'episodes' / episode / 'right.jpg'
    left.parent.mkdir(parents=True)
    left.write_bytes(shared_left)
    right.write_bytes(f'{name}-right'.encode())
    sample_id = f'{episode}:step:0'
    rows = [{
        'sample_id': sample_id,
        'episode_id': episode,
        'scenario_family': f'{name}_family',
        'model_input': {
            'overhead_images': {
                'left_rail_rgb': f'episodes/{episode}/left.jpg',
                'right_rail_rgb': f'episodes/{episode}/right.jpg',
            },
        },
        'traceability_metadata': {
            'scenario_id': episode,
            'spec_id': f'{name}_spec',
            'configuration_family_id': f'{name}_configuration',
            'geometry_fingerprint': f'{name}_geometry',
        },
    }]
    labels = [{
        'sample_id': sample_id,
        'episode_id': episode,
        'visual_state_labels': {
            'schema_version': 'room315.visual_state.v3',
            'shuttles': [{
                'id': 'L1',
                'presence': True,
                'loaded_state': 'empty',
                'location': {'block': 'A1', 'side': 'left'},
                'rail_position': {'s_ratio': s_ratio},
            }],
        },
    }]
    rows_path = root / 'rows.jsonl'
    labels_path = root / 'labels.jsonl'
    _write_jsonl(rows_path, rows)
    _write_jsonl(labels_path, labels)
    return audit.PartitionSpec(
        name=name,
        protocol='fixture',
        role='test',
        rows_path=rows_path,
        labels_path=labels_path,
        dataset_root=dataset,
        seeds=(123,),
    )


def test_presence_and_seed_overlap_are_separate_from_exact_samples(tmp_path):
    first = audit.build_partition_index(
        _partition(tmp_path, 'first', s_ratio=0.1)
    )
    second = audit.build_partition_index(
        _partition(tmp_path, 'second', s_ratio=0.2)
    )

    comparison = audit.compare_partitions(first, second)

    assert comparison['abstract_presence_configuration_overlap'] == {
        'left_unique_count': 1,
        'right_unique_count': 1,
        'overlap_count': 1,
        'examples': ['presence_001'],
        'interpretation': (
            'factor-level support overlap; it is not sample leakage by itself'
        ),
    }
    assert comparison['numeric_seed_overlap']['values'] == [123]
    assert comparison['exact_overlap_counts']['individual_image_sha256'] == 1
    assert comparison['exact_overlap_counts']['sample_ids'] == 0
    assert comparison['exact_overlap_counts']['full_label_row_sha256'] == 0
    assert comparison['exact_overlap_counts']['geometry_fingerprints'] == 0
    assert comparison['exact_overlap_counts']['source_scenario_ids'] is None
    assert comparison['exact_metric_coverage']['source_scenario_ids'][
        'comparison_status'
    ] == 'not_comparable_missing_field'


def test_legacy_stratification_geometry_is_indexed(tmp_path):
    spec = _partition(tmp_path, 'legacy_shape', s_ratio=0.1)
    rows = audit._read_jsonl(spec.rows_path)
    geometry = rows[0]['traceability_metadata'].pop('geometry_fingerprint')
    rows[0]['stratification_metadata'] = {
        'geometry_fingerprint': geometry,
    }
    _write_jsonl(spec.rows_path, rows)

    index = audit.build_partition_index(spec)

    assert index.metric_values['geometry_fingerprints'] == {geometry}
    assert index.metric_observation_counts['geometry_fingerprints'] == 1


def test_index_rejects_rows_without_matching_labels(tmp_path):
    spec = _partition(tmp_path, 'orphan', s_ratio=0.1)
    _write_jsonl(spec.labels_path, [])

    with pytest.raises(audit.AuditError, match='has no label'):
        audit.build_partition_index(spec)


def test_image_digest_claim_is_verified(tmp_path):
    spec = _partition(tmp_path, 'declared', s_ratio=0.1)
    rows = audit._read_jsonl(spec.rows_path)
    rows[0]['traceability_metadata']['source_images'] = {
        'left_rail_rgb': {
            'path': rows[0]['model_input']['overhead_images']['left_rail_rgb'],
            'sha256': '0' * 64,
        },
    }
    _write_jsonl(spec.rows_path, rows)

    with pytest.raises(audit.AuditError, match='image digest mismatch'):
        audit.build_partition_index(spec)


def test_portable_path_redacts_the_home_directory(monkeypatch, tmp_path):
    home = tmp_path / 'operator-home'
    monkeypatch.setattr(audit.Path, 'home', classmethod(lambda cls: home))

    assert audit._portable_path(home / 'dataset' / 'rows.jsonl') == (
        '~/dataset/rows.jsonl'
    )
    assert audit._portable_path(tmp_path / 'shared' / 'rows.jsonl') == str(
        (tmp_path / 'shared' / 'rows.jsonl').resolve()
    )
