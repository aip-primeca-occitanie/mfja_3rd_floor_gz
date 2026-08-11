"""Load gripper travel ranges and apply them to spawned robot descriptions.

The source SDF and URDF files remain immutable.  Each launch materializes an
SDF copy for Gazebo and an in-memory URDF string for robot_state_publisher so
both consumers use the same per-robot limits from the YAML configuration.
"""

import math
import os
import re
import tempfile
import xml.etree.ElementTree as ET

import yaml


SCHEMA_VERSION = 2
JAW_JOINT_NAMES = (
    'gripper_left_jaw_joint',
    'gripper_right_jaw_joint',
)
CONTROLLER_NAME = 'mfja::sim::systems::SymmetricGripperController'
RANGE_FIELDS = (
    'position_at_0_percent_m',
    'position_at_100_percent_m',
)
PERCENTAGE_FIELDS = (
    'default_open_percentage',
    'default_close_percentage',
)
ROOT_FIELDS = frozenset(('schema_version', 'grippers'))
ENTRY_FIELDS = frozenset(RANGE_FIELDS + PERCENTAGE_FIELDS)


def _finite_number(value, field_path):
    if isinstance(value, bool):
        raise RuntimeError(f'{field_path} must be a finite number.')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise RuntimeError(f'{field_path} must be a finite number.') from exc
    if not math.isfinite(number):
        raise RuntimeError(f'{field_path} must be a finite number.')
    return number


def _validate_gripper_entry(robot_name, raw_entry):
    entry_path = f'grippers.{robot_name}'
    if not isinstance(raw_entry, dict):
        raise RuntimeError(f'{entry_path} must be a YAML mapping.')

    unknown_fields = set(raw_entry) - ENTRY_FIELDS
    if unknown_fields:
        unknown = ', '.join(sorted(str(field) for field in unknown_fields))
        raise RuntimeError(
            f'{entry_path} contains unknown field(s): {unknown}.'
        )

    missing = [
        field
        for field in RANGE_FIELDS + PERCENTAGE_FIELDS
        if field not in raw_entry
    ]
    if missing:
        raise RuntimeError(
            f'{entry_path} is missing required field(s): {", ".join(missing)}.'
        )

    values = {
        field: _finite_number(raw_entry[field], f'{entry_path}.{field}')
        for field in RANGE_FIELDS + PERCENTAGE_FIELDS
    }
    q0 = values['position_at_0_percent_m']
    q100 = values['position_at_100_percent_m']
    if q0 < 0.0:
        raise RuntimeError(
            f'{entry_path}.position_at_0_percent_m must be greater than or '
            'equal to 0.'
        )
    if q100 <= q0:
        raise RuntimeError(
            f'{entry_path}.position_at_100_percent_m must be greater than '
            f'position_at_0_percent_m ({q0:.17g}).'
        )

    for field in PERCENTAGE_FIELDS:
        if not 0.0 <= values[field] <= 100.0:
            raise RuntimeError(
                f'{entry_path}.{field} must be between 0 and 100.'
            )

    return values


def load_gripper_range_config(config_path, robot_names):
    """Return validated plain dictionaries for requested robot instances only."""
    requested_names = list(dict.fromkeys(str(name).strip() for name in robot_names))
    if any(not name for name in requested_names):
        raise RuntimeError('Requested gripper robot names must not be empty.')
    if not requested_names:
        return {}

    try:
        with open(config_path, 'r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f'Cannot load gripper configuration "{config_path}": {exc}'
        ) from exc

    if not isinstance(document, dict):
        raise RuntimeError(
            f'Gripper configuration "{config_path}" must contain a YAML mapping.'
        )
    unknown_root_fields = set(document) - ROOT_FIELDS
    if unknown_root_fields:
        unknown = ', '.join(
            sorted(str(field) for field in unknown_root_fields)
        )
        raise RuntimeError(
            f'Gripper configuration "{config_path}" contains unknown '
            f'top-level field(s): {unknown}.'
        )
    schema_version = document.get('schema_version')
    if isinstance(schema_version, bool) or schema_version != SCHEMA_VERSION:
        raise RuntimeError(
            f'Gripper configuration "{config_path}" must set '
            f'schema_version: {SCHEMA_VERSION}.'
        )

    grippers = document.get('grippers')
    if not isinstance(grippers, dict):
        raise RuntimeError(
            f'Gripper configuration "{config_path}" must contain a '
            '"grippers" mapping.'
        )

    result = {}
    for robot_name in requested_names:
        if robot_name not in grippers:
            raise RuntimeError(
                f'Gripper configuration "{config_path}" is missing '
                f'grippers.{robot_name}.'
            )
        result[robot_name] = _validate_gripper_entry(
            robot_name,
            grippers[robot_name],
        )
    return result


def _single_element(root, xpath, description, source_path):
    matches = root.findall(xpath)
    if len(matches) != 1:
        raise RuntimeError(
            f'Expected exactly one {description} in "{source_path}", '
            f'found {len(matches)}.'
        )
    return matches[0]


def _required_child(element, child_path, description, source_path):
    child = element.find(child_path)
    if child is None:
        raise RuntimeError(
            f'Missing {description} in "{source_path}".'
        )
    return child


def _xml_number(value):
    return format(value, '.17g')


def _set_sdf_gripper_range(tree, source_path, q0, q100):
    root = tree.getroot()
    for joint_name in JAW_JOINT_NAMES:
        joint = _single_element(
            root,
            f".//joint[@name='{joint_name}']",
            f'SDF joint "{joint_name}"',
            source_path,
        )
        lower = _required_child(
            joint,
            'axis/limit/lower',
            f'<lower> for SDF joint "{joint_name}"',
            source_path,
        )
        upper = _required_child(
            joint,
            'axis/limit/upper',
            f'<upper> for SDF joint "{joint_name}"',
            source_path,
        )
        lower.text = _xml_number(q0)
        upper.text = _xml_number(q100)

    controller = _single_element(
        root,
        f".//plugin[@name='{CONTROLLER_NAME}']",
        f'SDF controller plugin "{CONTROLLER_NAME}"',
        source_path,
    )
    for element_name, value in (
        ('min_position', q0),
        ('max_position', q100),
        ('initial_position', q0),
    ):
        element = _required_child(
            controller,
            element_name,
            f'<{element_name}> in SDF gripper controller',
            source_path,
        )
        element.text = _xml_number(value)


def _set_urdf_gripper_range(tree, source_path, q0, q100):
    root = tree.getroot()
    for joint_name in JAW_JOINT_NAMES:
        joint = _single_element(
            root,
            f".//joint[@name='{joint_name}']",
            f'URDF joint "{joint_name}"',
            source_path,
        )
        limit = _required_child(
            joint,
            'limit',
            f'<limit> for URDF joint "{joint_name}"',
            source_path,
        )
        limit.set('lower', _xml_number(q0))
        limit.set('upper', _xml_number(q100))


def _temporary_sdf_path(robot_name):
    safe_name = re.sub(r'[^A-Za-z0-9_.-]+', '_', robot_name).strip('._-')
    safe_name = safe_name or 'industrial_robot'
    file_descriptor, output_path = tempfile.mkstemp(
        prefix=f'mfja_{safe_name}_gripper_range_',
        suffix='.sdf',
    )
    os.close(file_descriptor)
    return output_path


def materialize_gripper_assets(
    model_sdf_path,
    urdf_path,
    robot_name,
    range_config,
    *,
    output_sdf_path=None,
):
    """Create a configured SDF copy and matching in-memory URDF description."""
    validated = _validate_gripper_entry(robot_name, range_config)
    q0 = validated['position_at_0_percent_m']
    q100 = validated['position_at_100_percent_m']

    try:
        sdf_tree = ET.parse(model_sdf_path)
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(f'Cannot parse robot SDF "{model_sdf_path}": {exc}') from exc
    try:
        urdf_tree = ET.parse(urdf_path)
    except (OSError, ET.ParseError) as exc:
        raise RuntimeError(f'Cannot parse robot URDF "{urdf_path}": {exc}') from exc

    _set_sdf_gripper_range(sdf_tree, model_sdf_path, q0, q100)
    _set_urdf_gripper_range(urdf_tree, urdf_path, q0, q100)

    output_path = output_sdf_path or _temporary_sdf_path(robot_name)
    try:
        sdf_tree.write(output_path, encoding='utf-8', xml_declaration=True)
    except OSError as exc:
        raise RuntimeError(
            f'Cannot write configured robot SDF "{output_path}": {exc}'
        ) from exc

    return {
        'sdf_path': output_path,
        'robot_description': ET.tostring(
            urdf_tree.getroot(),
            encoding='unicode',
        ),
        **validated,
    }
