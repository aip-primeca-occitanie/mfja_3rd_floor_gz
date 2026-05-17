#!/usr/bin/env python3

import argparse
import math
import os
import re
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


@dataclass
class SwitchPose:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


SWITCH_DEFINITIONS = (
    {
        'entity_name': 'A1_droit_switch',
        'summary_label': 'A1R',
        'logical_station': 'A1',
        'side_fr': 'droit',
        'side_en': 'right',
        'side_short': 'r',
        'petit_yaw': 2.08919,
        'grand_yaw': -2.1,
    },
    {
        'entity_name': 'A2_droit_switch',
        'summary_label': 'A2R',
        'logical_station': 'A2',
        'side_fr': 'droit',
        'side_en': 'right',
        'side_short': 'r',
        'petit_yaw': 2.60106,
        'grand_yaw': 0.50666,
    },
    {
        'entity_name': 'A3_droit_switch',
        'summary_label': 'A3R',
        'logical_station': 'A3',
        'side_fr': 'droit',
        'side_en': 'right',
        'side_short': 'r',
        'petit_yaw': -1.5877,
        'grand_yaw': 0.50666,
    },
    {
        'entity_name': 'A4_droit_switch',
        'summary_label': 'A4R',
        'logical_station': 'A4',
        'side_fr': 'droit',
        'side_en': 'right',
        'side_short': 'r',
        'petit_yaw': -1.0587,
        'grand_yaw': 3.13,
    },
    {
        'entity_name': 'A1_gauche_switch',
        'summary_label': 'A1L',
        'logical_station': 'A1',
        'side_fr': 'gauche',
        'side_en': 'left',
        'side_short': 'l',
        'petit_yaw': 1.55349,
        'grand_yaw': -2.63653,
    },
    {
        'entity_name': 'A2_gauche_switch',
        'summary_label': 'A2L',
        'logical_station': 'A2',
        'side_fr': 'gauche',
        'side_en': 'left',
        'side_short': 'l',
        'petit_yaw': 2.08919,
        'grand_yaw': -0.025,
    },
    {
        'entity_name': 'A3_gauche_switch',
        'summary_label': 'A3L',
        'logical_station': 'A3',
        'side_fr': 'gauche',
        'side_en': 'left',
        'side_short': 'l',
        'petit_yaw': -1.0587,
        'grand_yaw': 1.04,
    },
    {
        'entity_name': 'A4_gauche_switch',
        'summary_label': 'A4L',
        'logical_station': 'A4',
        'side_fr': 'gauche',
        'side_en': 'left',
        'side_short': 'l',
        'petit_yaw': -0.540815,
        'grand_yaw': -2.63653,
    },
)

MODE_YAWS: Dict[str, Dict[str, float]] = {
    'petit_boucle': {
        definition['entity_name']: definition['petit_yaw']
        for definition in SWITCH_DEFINITIONS
    },
    'grand_boucle': {
        definition['entity_name']: definition['grand_yaw']
        for definition in SWITCH_DEFINITIONS
    },
}

SWITCH_ORDER = tuple(
    definition['entity_name'] for definition in SWITCH_DEFINITIONS
)
ENTITY_TO_SUMMARY_LABEL = {
    definition['entity_name']: definition['summary_label']
    for definition in SWITCH_DEFINITIONS
}

MODE_ALIASES = {
    'grand_boucle': 'grand_boucle',
    'grand': 'grand_boucle',
    'grande_boucle': 'grand_boucle',
    'large': 'grand_boucle',
    'big': 'grand_boucle',
    'e': 'grand_boucle',
    'exterior': 'grand_boucle',
    '1': 'grand_boucle',
    'petit_boucle': 'petit_boucle',
    'petit': 'petit_boucle',
    'small': 'petit_boucle',
    'mini': 'petit_boucle',
    'i': 'petit_boucle',
    'interior': 'petit_boucle',
    '0': 'petit_boucle',
}

MIXED_MODE = 'mixed'
SWITCH_VISUAL_PATH = 'blade_link::blade_visual'
SWITCH_MODE_COLORS = {
    'petit_boucle': {
        'ambient': (0.05, 0.85, 0.15, 1.0),
        'diffuse': (0.05, 0.85, 0.15, 1.0),
        'specular': (0.02, 0.20, 0.05, 1.0),
        'emissive': (0.00, 0.03, 0.00, 1.0),
    },
    'grand_boucle': {
        'ambient': (1.00, 0.62, 0.05, 1.0),
        'diffuse': (1.00, 0.62, 0.05, 1.0),
        'specular': (0.25, 0.12, 0.02, 1.0),
        'emissive': (0.04, 0.02, 0.00, 1.0),
    },
}


def _normalize_token(raw_value: str) -> str:
    return re.sub(r'[^a-z0-9]+', '_', raw_value.strip().lower()).strip('_')


def _build_switch_selector_aliases() -> Dict[str, Tuple[str, ...]]:
    aliases: Dict[str, Tuple[str, ...]] = {}
    right_switches = []
    left_switches = []
    station_switches: Dict[str, List[str]] = {}

    for definition in SWITCH_DEFINITIONS:
        switch_name = definition['entity_name']
        summary_label = definition['summary_label']
        station = summary_label[:-1].lower()
        side_fr = definition['side_fr']
        side_en = definition['side_en']
        side_short = definition['side_short']
        normalized_aliases = {
            _normalize_token(switch_name),
            _normalize_token(summary_label),
            _normalize_token(f'{station}_{side_fr}'),
            _normalize_token(f'{station}_{side_en}'),
            _normalize_token(f'{station}{side_short}'),
            _normalize_token(f'{station}_{side_short}'),
        }
        for alias in normalized_aliases:
            aliases[alias] = (switch_name,)

        if side_fr == 'droit':
            right_switches.append(switch_name)
        else:
            left_switches.append(switch_name)
        station_switches.setdefault(definition['logical_station'].lower(), []).append(
            switch_name
        )

    aliases['all'] = tuple(SWITCH_ORDER)
    aliases['right'] = tuple(right_switches)
    aliases['droit'] = tuple(right_switches)
    aliases['left'] = tuple(left_switches)
    aliases['gauche'] = tuple(left_switches)
    for station, switches in station_switches.items():
        aliases[station] = tuple(switches)

    return aliases


SWITCH_SELECTOR_ALIASES = _build_switch_selector_aliases()


def _canonical_mode_label(mode: str) -> str:
    return mode.upper()


def _normalize_mode(raw_value: str) -> Optional[str]:
    normalized = _normalize_token(raw_value)
    return MODE_ALIASES.get(normalized)


def _normalize_initial_loop_mode(raw_value: str) -> Optional[str]:
    normalized = _normalize_token(raw_value)
    if normalized in ('', 'auto', 'world', 'layout', 'detect', 'detected'):
        return None
    return MODE_ALIASES.get(normalized)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _angles_match(first: float, second: float, tolerance: float = 0.03) -> bool:
    return abs(_normalize_angle(first - second)) <= tolerance


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def _resolve_world_file(path_hint: str, world_name: str) -> str:
    if path_hint:
        return path_hint if os.path.isabs(path_hint) else os.path.abspath(path_hint)

    description_pkg = get_package_share_directory('mfja_3rd_floor_description')
    world_file_stem = 'mfja_3rd_floor' if world_name == 'default' else world_name
    return os.path.join(description_pkg, 'worlds', world_file_stem + '.world')


def _parse_pose(pose_text: str) -> SwitchPose:
    values = [float(token) for token in pose_text.split()]
    if len(values) == 3:
        values.extend([0.0, 0.0, 0.0])
    if len(values) != 6:
        raise RuntimeError(
            f'Expected pose with 3 or 6 values, but received: "{pose_text}"'
        )
    return SwitchPose(*values)


def _load_switch_layout(world_file: str) -> Dict[str, SwitchPose]:
    if not os.path.exists(world_file):
        raise RuntimeError(f'World file does not exist: {world_file}')

    tree = ET.parse(world_file)
    root = tree.getroot()
    world_element = root.find('world')
    if world_element is None:
        raise RuntimeError(f'No <world> element found in: {world_file}')

    layout = {}
    for include in world_element.findall('include'):
        name_element = include.find('name')
        pose_element = include.find('pose')
        if name_element is None or pose_element is None:
            continue
        entity_name = (name_element.text or '').strip()
        if entity_name not in SWITCH_ORDER:
            continue
        layout[entity_name] = _parse_pose(pose_element.text or '')

    return layout


def _detect_switch_mode(switch_name: str, yaw: float) -> Optional[str]:
    for mode_name, target_yaws in MODE_YAWS.items():
        target_yaw = target_yaws.get(switch_name)
        if target_yaw is not None and _angles_match(yaw, target_yaw):
            return mode_name
    return None


def _detect_switch_states_from_layout(
    layout: Dict[str, SwitchPose],
    switch_names: List[str],
) -> Dict[str, Optional[str]]:
    states = {}
    for switch_name in switch_names:
        switch_pose = layout.get(switch_name)
        states[switch_name] = (
            _detect_switch_mode(switch_name, switch_pose.yaw)
            if switch_pose is not None
            else None
        )
    return states


def _summarize_switch_states(
    switch_states: Dict[str, Optional[str]],
    switch_names: List[str],
) -> Optional[str]:
    if not switch_names:
        return None

    modes = [switch_states.get(switch_name) for switch_name in switch_names]
    if any(mode is None for mode in modes):
        return None

    unique_modes = set(modes)
    if len(unique_modes) == 1:
        return unique_modes.pop()
    return MIXED_MODE


def _format_switch_states(
    switch_states: Dict[str, Optional[str]],
    switch_names: List[str],
) -> str:
    return ', '.join(
        f'{ENTITY_TO_SUMMARY_LABEL.get(switch_name, switch_name)}='
        f'{_canonical_mode_label(switch_states.get(switch_name)) if switch_states.get(switch_name) is not None else "UNKNOWN"}'
        for switch_name in switch_names
    )


class ConveyorLoopModeController(Node):
    def __init__(self, args):
        super().__init__('conveyor_loop_mode_controller')
        self.world_name = args.world
        self.world_file = _resolve_world_file(args.world_file, self.world_name)
        self.partition = args.partition
        self.timeout_ms = args.timeout_ms
        self.retries = args.retries
        self.initial_loop_mode = _normalize_initial_loop_mode(args.initial_loop_mode)
        self.keep_paused_after_initial_loop = args.keep_paused_after_initial_loop
        self.pause_during_switch_update = args.pause_during_switch_update
        if self.initial_loop_mode is None and args.initial_loop_mode.strip().lower() not in (
            '',
            'auto',
            'world',
            'layout',
            'detect',
            'detected',
        ):
            raise RuntimeError(
                f'Unsupported initial loop mode "{args.initial_loop_mode}". '
                'Use auto, INTERIOR, EXTERIOR, PETIT_BOUCLE, or GRAND_BOUCLE.'
            )
        self.command_topic = args.command_topic
        self.state_topic = args.state_topic
        self.switch_command_topic = args.switch_command_topic
        self.switch_state_topic = args.switch_state_topic

        self.switch_layout = _load_switch_layout(self.world_file)
        self.managed_switches = [
            switch_name for switch_name in SWITCH_ORDER
            if switch_name in self.switch_layout
        ]
        self.switch_visual_ids: Dict[str, int] = {}
        self.current_switch_states = _detect_switch_states_from_layout(
            self.switch_layout,
            self.managed_switches,
        )
        self.current_mode = _summarize_switch_states(
            self.current_switch_states,
            self.managed_switches,
        )

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.state_publisher = self.create_publisher(String, self.state_topic, state_qos)
        self.switch_state_publisher = self.create_publisher(
            String,
            self.switch_state_topic,
            state_qos,
        )
        self.command_subscription = self.create_subscription(
            String, self.command_topic, self._handle_command, 10
        )
        self.switch_command_subscription = self.create_subscription(
            String,
            self.switch_command_topic,
            self._handle_switch_command,
            10,
        )

        if self.managed_switches:
            self.get_logger().info(
                f'Loaded {len(self.managed_switches)} managed switch poses from {self.world_file}.'
            )
        else:
            self.get_logger().warning(
                'No managed loop switches were found in the selected world file. '
                'Incoming commands will be ignored.'
            )

        missing_switches = [
            switch_name for switch_name in SWITCH_ORDER
            if switch_name not in self.switch_layout
        ]
        if missing_switches:
            self.get_logger().warning(
                'Missing managed switches in world layout: ' + ', '.join(missing_switches)
            )

        if self.current_mode is not None:
            self.get_logger().info(
                'Detected initial switch layout from world file: '
                f'{_canonical_mode_label(self.current_mode)}'
            )
        else:
            self.get_logger().info(
                'Initial world layout does not resolve to a single global loop mode. '
                f'Listening on {self.command_topic} and {self.switch_command_topic} '
                'for explicit switch commands.'
            )
        self._publish_current_switch_state()
        self.get_logger().info(
            'Per-switch command topic: '
            f'{self.switch_command_topic} '
            '(examples: "A1R=INTERIOR", '
            '"A1=EXTERIOR", "RIGHT=INTERIOR, A3L=EXTERIOR").'
        )

        if self.initial_loop_mode is not None:
            self._apply_mode(
                self.initial_loop_mode,
                source='initial_loop_mode launch argument',
                resume_after=not self.keep_paused_after_initial_loop,
            )

    def _handle_command(self, msg: String):
        requested_mode = _normalize_mode(msg.data)
        if requested_mode is None:
            self.get_logger().warning(
                f'Unsupported loop mode command "{msg.data}". '
                'Use INTERIOR, EXTERIOR, PETIT_BOUCLE, or GRAND_BOUCLE.'
            )
            return

        self._apply_mode(requested_mode, source=f'topic command "{msg.data}"')

    def _handle_switch_command(self, msg: String):
        requested_switch_modes = self._parse_switch_command(msg.data)
        if not requested_switch_modes:
            return

        self._apply_switch_modes(
            requested_switch_modes,
            source=f'switch topic command "{msg.data}"',
        )

    def _parse_switch_command(self, raw_command: str) -> Optional[Dict[str, str]]:
        command = raw_command.strip()
        if not command:
            self.get_logger().warning(
                f'Ignoring empty command on {self.switch_command_topic}.'
            )
            return None

        requested_mode = _normalize_mode(command)
        if requested_mode is not None:
            return {
                switch_name: requested_mode
                for switch_name in self.managed_switches
            }

        requested_switch_modes: Dict[str, str] = {}
        entries = [entry.strip() for entry in re.split(r'[,\n;]+', command) if entry.strip()]
        if not entries:
            self.get_logger().warning(
                f'Could not parse any switch assignment from "{raw_command}".'
            )
            return None

        for entry in entries:
            selector_text = ''
            mode_text = ''

            for separator in ('=', ':'):
                if separator in entry:
                    selector_text, mode_text = entry.split(separator, 1)
                    break

            if not selector_text and not mode_text:
                parts = entry.split()
                if len(parts) == 1:
                    requested_mode = _normalize_mode(parts[0])
                    if requested_mode is None:
                        self.get_logger().warning(
                            f'Unsupported switch command fragment "{entry}". '
                            'Use forms like "A1R=INTERIOR" or "RIGHT EXTERIOR".'
                        )
                        return None
                    selector_text = 'all'
                    mode_text = parts[0]
                elif len(parts) == 2:
                    selector_text, mode_text = parts
                else:
                    self.get_logger().warning(
                        f'Unsupported switch command fragment "{entry}". '
                        'Use forms like "A1R=INTERIOR" or "RIGHT EXTERIOR".'
                    )
                    return None

            requested_mode = _normalize_mode(mode_text)
            if requested_mode is None:
                self.get_logger().warning(
                    f'Unsupported switch target mode "{mode_text}" in "{entry}". '
                    'Use INTERIOR, EXTERIOR, PETIT_BOUCLE, or GRAND_BOUCLE.'
                )
                return None

            selected_switches = self._resolve_switch_selector(selector_text)
            if not selected_switches:
                self.get_logger().warning(
                    f'Unsupported switch selector "{selector_text}" in "{entry}". '
                    'Use exact Gazebo names like A1_droit_switch, '
                    'short aliases like A1R / A3L, '
                    'row aliases like A1 / A2 / A3 / A4, '
                    'or group aliases ALL / RIGHT / LEFT.'
                )
                return None

            for switch_name in selected_switches:
                requested_switch_modes[switch_name] = requested_mode

        return requested_switch_modes

    def _resolve_switch_selector(self, selector_text: str) -> List[str]:
        normalized_selector = _normalize_token(selector_text)
        selected_switches = SWITCH_SELECTOR_ALIASES.get(normalized_selector, ())
        return [
            switch_name for switch_name in selected_switches
            if switch_name in self.managed_switches
        ]

    def _apply_mode(
        self,
        mode: str,
        source: str,
        resume_after: bool = True,
    ):
        self._apply_switch_modes(
            {
                switch_name: mode
                for switch_name in self.managed_switches
            },
            source=source,
            resume_after=resume_after,
        )

    def _apply_switch_modes(
        self,
        requested_switch_modes: Dict[str, str],
        source: str,
        resume_after: bool = True,
    ):
        if not self.managed_switches:
            self.get_logger().warning(
                f'Ignoring {source} because no managed switches are available.'
            )
            return

        if not requested_switch_modes:
            self.get_logger().warning(
                f'Ignoring {source} because it did not target any managed switch.'
            )
            return

        targeted_switches = {
            switch_name: mode
            for switch_name, mode in requested_switch_modes.items()
            if switch_name in self.managed_switches
        }
        if not targeted_switches:
            self.get_logger().warning(
                f'Ignoring {source} because it did not match any available switch.'
            )
            return

        self.get_logger().info(
            'Applying switch command from '
            f'{source}: '
            + ', '.join(
                f'{switch_name}={_canonical_mode_label(mode)}'
                for switch_name, mode in targeted_switches.items()
            )
        )

        world_was_paused_by_controller = False
        if self.pause_during_switch_update:
            pause_ok, pause_output = self._set_world_pause(True)
            if pause_ok:
                world_was_paused_by_controller = True
            else:
                pause_details = pause_output or 'no diagnostic output returned by gz service'
                self.get_logger().warning(
                    'Failed to pause the Gazebo world before switching modes. '
                    f'Continuing anyway: {pause_details}'
                )

        try:
            failures = self._set_switches_in_place(targeted_switches)
        finally:
            if world_was_paused_by_controller and resume_after:
                resume_ok, resume_output = self._set_world_pause(False)
                if not resume_ok:
                    resume_details = (
                        resume_output or 'no diagnostic output returned by gz service'
                    )
                    self.get_logger().error(
                        'Failed to resume the Gazebo world after switching modes: '
                        f'{resume_details}'
                    )
            elif world_was_paused_by_controller:
                self.get_logger().info(
                    'Leaving the Gazebo world paused after applying the initial loop mode.'
                )

        self._refresh_current_switch_state()
        if failures:
            for switch_name, output in failures:
                details = output or 'no diagnostic output returned by gz service'
                self.get_logger().error(
                    f'Failed to apply '
                    f'{_canonical_mode_label(targeted_switches[switch_name])} '
                    f'to {switch_name}: {details}'
                )
            return

        self.get_logger().info(
            'Applied switch command successfully. Current global state: '
            f'{_canonical_mode_label(self.current_mode) if self.current_mode else "UNKNOWN"}.'
        )

    def _build_entity_set_pose_command(
        self,
        entity_name: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        timeout_ms: Optional[int] = None,
    ):
        qx, qy, qz, qw = _quaternion_from_rpy(roll, pitch, yaw)
        request = (
            f'name: "{entity_name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
        )
        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms if timeout_ms is not None else self.timeout_ms),
            '--req', request,
        ]
        return command

    def _build_set_pose_command(self, switch_name: str, switch_pose: SwitchPose, target_yaw: float):
        return self._build_entity_set_pose_command(
            entity_name=switch_name,
            x=switch_pose.x,
            y=switch_pose.y,
            z=switch_pose.z,
            roll=switch_pose.roll,
            pitch=switch_pose.pitch,
            yaw=target_yaw,
        )

    @staticmethod
    def _proto_string(value: str) -> str:
        return value.replace('\\', '\\\\').replace('"', '\\"')

    @staticmethod
    def _switch_model_sdf(mode: str) -> str:
        colors = SWITCH_MODE_COLORS[mode]

        def rgba(name: str) -> str:
            red, green, blue, alpha = colors[name]
            return f'{red:.3f} {green:.3f} {blue:.3f} {alpha:.3f}'

        return (
            "<sdf version='1.9'>"
            "<model name='rail_switch_3pos_droit'>"
            "<static>true</static>"
            "<link name='blade_link'>"
            "<kinematic>true</kinematic>"
            "<gravity>false</gravity>"
            "<inertial><mass>0.1</mass><inertia>"
            "<ixx>0.001</ixx><ixy>0.0</ixy><ixz>0.0</ixz>"
            "<iyy>0.001</iyy><iyz>0.0</iyz><izz>0.001</izz>"
            "</inertia></inertial>"
            "<visual name='blade_visual'>"
            "<geometry><mesh>"
            "<uri>model://rail_switch_3pos_droit/meshes/aiguillage3.stl</uri>"
            "<scale>39.37 39.37 39.37</scale>"
            "</mesh></geometry>"
            "<material>"
            f"<ambient>{rgba('ambient')}</ambient>"
            f"<diffuse>{rgba('diffuse')}</diffuse>"
            f"<specular>{rgba('specular')}</specular>"
            f"<emissive>{rgba('emissive')}</emissive>"
            "</material>"
            "</visual>"
            "<collision name='blade_collision'>"
            "<geometry><mesh>"
            "<uri>model://rail_switch_3pos_droit/meshes/aiguillage3.stl</uri>"
            "<scale>39.37 39.37 39.37</scale>"
            "</mesh></geometry>"
            "<surface><friction><ode><mu>0.35</mu><mu2>0.35</mu2></ode>"
            "<bullet><friction>0.35</friction><friction2>0.35</friction2>"
            "<rolling_friction>0.005</rolling_friction></bullet></friction>"
            "<contact><collide_bitmask>0x0001</collide_bitmask>"
            "<ode><kp>2000000</kp><kd>50</kd><max_vel>0.1</max_vel>"
            "<min_depth>0.001</min_depth></ode>"
            "<bullet><kp>2000000</kp><kd>50</kd></bullet>"
            "</contact></surface>"
            "</collision>"
            "</link>"
            "</model>"
            "</sdf>"
        )

    def _build_remove_switch_command(
        self,
        switch_name: str,
        timeout_ms: Optional[int] = None,
    ):
        return [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/remove',
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms if timeout_ms is not None else self.timeout_ms),
            '--req', f'name: "{switch_name}", type: MODEL',
        ]

    def _build_create_switch_command(
        self,
        switch_name: str,
        switch_pose: SwitchPose,
        target_yaw: float,
        mode: str,
        timeout_ms: Optional[int] = None,
    ):
        qx, qy, qz, qw = _quaternion_from_rpy(
            switch_pose.roll,
            switch_pose.pitch,
            target_yaw,
        )
        sdf = self._proto_string(self._switch_model_sdf(mode))
        request = (
            f'sdf: "{sdf}", '
            f'name: "{switch_name}", '
            'allow_renaming: false, '
            'relative_to: "world", '
            'pose { '
            f'position {{x: {switch_pose.x}, y: {switch_pose.y}, z: {switch_pose.z}}} '
            f'orientation {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}} '
            '}'
        )
        return [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms if timeout_ms is not None else self.timeout_ms),
            '--req', request,
        ]

    def _build_visual_config_command(
        self,
        switch_name: str,
        visual_id: int,
        mode: str,
        timeout_ms: Optional[int] = None,
    ):
        colors = SWITCH_MODE_COLORS[mode]

        def color_block(name: str) -> str:
            red, green, blue, alpha = colors[name]
            return (
                f'{name} {{ r: {red:.3f} g: {green:.3f} '
                f'b: {blue:.3f} a: {alpha:.3f} }}'
            )

        request = (
            f'id: {visual_id} '
            'material { '
            f'{color_block("ambient")} '
            f'{color_block("diffuse")} '
            f'{color_block("specular")} '
            f'{color_block("emissive")} '
            '}'
        )
        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/visual_config',
            '--reqtype', 'gz.msgs.Visual',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(timeout_ms if timeout_ms is not None else self.timeout_ms),
            '--req', request,
        ]
        return command

    def _build_scene_graph_command(self, timeout_ms: Optional[int] = None):
        return [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/scene/graph',
            '--reqtype', 'gz.msgs.Empty',
            '--reptype', 'gz.msgs.StringMsg',
            '--timeout', str(timeout_ms if timeout_ms is not None else self.timeout_ms),
            '--req', '',
        ]

    @staticmethod
    def _decode_scene_graph_response(output: str) -> str:
        match = re.search(r'data:\s*"(.*)"\s*$', output, flags=re.DOTALL)
        if not match:
            return output
        return bytes(match.group(1), 'utf-8').decode('unicode_escape')

    @staticmethod
    def _visual_ids_from_scene_graph(graph_text: str) -> Dict[str, int]:
        labels: Dict[int, str] = {}
        children: Dict[int, List[int]] = {}
        for match in re.finditer(r'^\s*(\d+)\s+\[label="(.+?) \((\d+)\)"\];', graph_text, re.MULTILINE):
            node_id = int(match.group(1))
            labels[node_id] = match.group(2)
        for match in re.finditer(r'^\s*(\d+)\s*->\s*(\d+)\s*\[', graph_text, re.MULTILINE):
            parent_id = int(match.group(1))
            child_id = int(match.group(2))
            children.setdefault(parent_id, []).append(child_id)

        switch_visual_ids: Dict[str, int] = {}
        for model_id, label in labels.items():
            if label not in SWITCH_ORDER:
                continue
            link_ids = [
                child_id for child_id in children.get(model_id, [])
                if labels.get(child_id) == 'blade_link'
            ]
            for link_id in link_ids:
                visual_ids = [
                    child_id for child_id in children.get(link_id, [])
                    if labels.get(child_id) == 'blade_visual'
                ]
                if visual_ids:
                    switch_visual_ids[label] = visual_ids[0]
                    break
        return switch_visual_ids

    def _refresh_switch_visual_ids(self) -> bool:
        command = self._build_scene_graph_command()
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition
        completed = subprocess.run(
            command,
            check=False,
            env=environment,
            text=True,
            capture_output=True,
        )
        output = '\n'.join(
            part for part in [completed.stdout, completed.stderr] if part
        ).strip()
        if completed.returncode != 0 or 'timed out' in output.lower():
            return False
        visual_ids = self._visual_ids_from_scene_graph(
            self._decode_scene_graph_response(output)
        )
        if not visual_ids:
            return False
        self.switch_visual_ids.update(visual_ids)
        return True

    def _set_world_pause(self, paused: bool):
        command = [
            'gz', 'service',
            '-s', f'/world/{self.world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', str(self.timeout_ms),
            '--req', f'pause: {"true" if paused else "false"}',
        ]
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition

        for _ in range(self.retries):
            completed = subprocess.run(
                command,
                check=False,
                env=environment,
                text=True,
                capture_output=True,
            )
            output = '\n'.join(
                part for part in [completed.stdout, completed.stderr] if part
            ).strip()
            lowered_output = output.lower()
            timed_out = 'timed out' in lowered_output
            returned_false = 'data: false' in lowered_output
            success = completed.returncode == 0 and not timed_out and not returned_false
            if success:
                return True, output

        return False, output

    def _run_gz_command(self, command: List[str]):
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition
        completed = subprocess.run(
            command,
            check=False,
            env=environment,
            text=True,
            capture_output=True,
        )
        output = '\n'.join(
            part for part in [completed.stdout, completed.stderr] if part
        ).strip()
        lowered_output = output.lower()
        timed_out = 'timed out' in lowered_output
        returned_false = 'data: false' in lowered_output
        success = completed.returncode == 0 and not timed_out and not returned_false
        return success, output

    def _recreate_switch_model(
        self,
        switch_name: str,
        switch_pose: SwitchPose,
        target_yaw: float,
        mode: str,
    ):
        remove_ok, remove_output = self._run_gz_command(
            self._build_remove_switch_command(switch_name)
        )
        if not remove_ok:
            return False, remove_output

        create_ok, create_output = self._run_gz_command(
            self._build_create_switch_command(
                switch_name=switch_name,
                switch_pose=switch_pose,
                target_yaw=target_yaw,
                mode=mode,
            )
        )
        return create_ok, create_output

    def _set_switch_models(self, requested_switch_modes: Dict[str, str]):
        pending = {
            switch_name: (
                self.switch_layout[switch_name],
                MODE_YAWS[mode][switch_name],
                mode,
            )
            for switch_name, mode in requested_switch_modes.items()
            if switch_name in self.switch_layout
        }
        failures = {}

        for attempt in range(1, self.retries + 1):
            if not pending:
                break

            next_pending = {}
            failures.clear()

            for switch_name, (switch_pose, target_yaw, mode) in pending.items():
                success, output = self._recreate_switch_model(
                    switch_name=switch_name,
                    switch_pose=switch_pose,
                    target_yaw=target_yaw,
                    mode=mode,
                )
                if success:
                    self.switch_layout[switch_name] = SwitchPose(
                        x=switch_pose.x,
                        y=switch_pose.y,
                        z=switch_pose.z,
                        roll=switch_pose.roll,
                        pitch=switch_pose.pitch,
                        yaw=target_yaw,
                    )
                    self.switch_visual_ids.pop(switch_name, None)
                    continue

                failures[switch_name] = output
                next_pending[switch_name] = (switch_pose, target_yaw, mode)

            pending = next_pending
            if pending and attempt < self.retries:
                self.get_logger().warning(
                    f'Switch model recreate round {attempt} failed for '
                    f'{len(pending)} switch(es); retrying the remaining requests.'
                )

        return [(switch_name, failures.get(switch_name, '')) for switch_name in pending]

    def _set_switches_in_place(self, requested_switch_modes: Dict[str, str]):
        # Keep the switch model visible while changing state; recreate only if
        # Gazebo rejects the direct pose or material update.
        pose_failures = dict(self._set_switch_poses_parallel(requested_switch_modes))
        color_targets = {
            switch_name: mode
            for switch_name, mode in requested_switch_modes.items()
            if switch_name not in pose_failures
        }
        color_failures = dict(self._set_switch_materials_parallel(color_targets))
        fallback_targets = {
            switch_name: requested_switch_modes[switch_name]
            for switch_name in set(pose_failures) | set(color_failures)
        }
        if not fallback_targets:
            return []

        self.get_logger().warning(
            'Direct switch pose/color update failed for '
            f'{len(fallback_targets)} switch(es); recreating only those switches as fallback.'
        )
        return self._set_switch_models(fallback_targets)

    def _set_switch_materials_parallel(self, requested_switch_modes: Dict[str, str]):
        target_modes = {
            switch_name: mode
            for switch_name, mode in requested_switch_modes.items()
            if switch_name in self.switch_layout and mode in SWITCH_MODE_COLORS
        }
        pending = dict(target_modes)
        failures = {}
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition

        for attempt in range(1, self.retries + 1):
            if not pending:
                break
            missing_visual_ids = [
                switch_name for switch_name in pending
                if switch_name not in self.switch_visual_ids
            ]
            if missing_visual_ids:
                self._refresh_switch_visual_ids()

            launched = {}
            for switch_name, mode in pending.items():
                visual_id = self.switch_visual_ids.get(switch_name)
                if visual_id is None:
                    continue
                command = self._build_visual_config_command(
                    switch_name=switch_name,
                    visual_id=visual_id,
                    mode=mode,
                )
                launched[switch_name] = (
                    subprocess.Popen(
                        command,
                        env=environment,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                    ),
                    mode,
                )

            next_pending = {
                switch_name: mode
                for switch_name, mode in pending.items()
                if switch_name not in launched
            }
            failures = {
                switch_name: (
                    f'Could not resolve Gazebo visual id for '
                    f'{switch_name}::{SWITCH_VISUAL_PATH}.'
                )
                for switch_name in pending
                if switch_name not in launched
            }

            for switch_name, (process, mode) in launched.items():
                stdout, stderr = process.communicate()
                output = '\n'.join(part for part in [stdout, stderr] if part).strip()
                lowered_output = output.lower()
                timed_out = 'timed out' in lowered_output
                returned_false = 'data: false' in lowered_output
                success = process.returncode == 0 and not timed_out and not returned_false

                if success:
                    continue

                failures[switch_name] = output
                next_pending[switch_name] = mode

            pending = next_pending
            if pending and attempt < self.retries:
                self.get_logger().warning(
                    f'Switch color update round {attempt} failed for '
                    f'{len(pending)} switch(es); retrying the remaining requests.'
                )

        return [(switch_name, failures.get(switch_name, '')) for switch_name in pending]

    def _set_switch_poses_parallel(self, requested_switch_modes: Dict[str, str]):
        pending = {
            switch_name: (
                self.switch_layout[switch_name],
                MODE_YAWS[mode][switch_name],
            )
            for switch_name, mode in requested_switch_modes.items()
            if switch_name in self.switch_layout
        }
        failures = {}
        environment = os.environ.copy()
        environment['GZ_PARTITION'] = self.partition

        for attempt in range(1, self.retries + 1):
            if not pending:
                break

            launched = {}
            for switch_name, (switch_pose, target_yaw) in pending.items():
                command = self._build_set_pose_command(
                    switch_name=switch_name,
                    switch_pose=switch_pose,
                    target_yaw=target_yaw,
                )
                launched[switch_name] = (
                    subprocess.Popen(
                        command,
                        env=environment,
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE,
                    ),
                    switch_pose,
                    target_yaw,
                )

            next_pending = {}
            failures.clear()

            for switch_name, (process, switch_pose, target_yaw) in launched.items():
                stdout, stderr = process.communicate()
                output = '\n'.join(part for part in [stdout, stderr] if part).strip()
                lowered_output = output.lower()
                timed_out = 'timed out' in lowered_output
                returned_false = 'data: false' in lowered_output
                success = process.returncode == 0 and not timed_out and not returned_false

                if success:
                    self.switch_layout[switch_name] = SwitchPose(
                        x=switch_pose.x,
                        y=switch_pose.y,
                        z=switch_pose.z,
                        roll=switch_pose.roll,
                        pitch=switch_pose.pitch,
                        yaw=target_yaw,
                    )
                    continue

                failures[switch_name] = output
                next_pending[switch_name] = (switch_pose, target_yaw)

            pending = next_pending
            if pending and attempt < self.retries:
                self.get_logger().warning(
                    f'Parallel switch update round {attempt} failed for '
                    f'{len(pending)} switch(es); retrying the remaining requests.'
                )

        return [(switch_name, failures.get(switch_name, '')) for switch_name in pending]

    def _refresh_current_switch_state(self):
        self.current_switch_states = _detect_switch_states_from_layout(
            self.switch_layout,
            self.managed_switches,
        )
        self.current_mode = _summarize_switch_states(
            self.current_switch_states,
            self.managed_switches,
        )
        self._publish_current_switch_state()

    def _publish_current_switch_state(self):
        current_label = (
            _canonical_mode_label(self.current_mode)
            if self.current_mode is not None
            else 'UNKNOWN'
        )
        self._publish_state_label(current_label)
        self._publish_switch_state_summary()

    def _publish_state_label(self, label: str):
        msg = String()
        msg.data = label
        self.state_publisher.publish(msg)

    def _publish_switch_state_summary(self):
        msg = String()
        msg.data = _format_switch_states(
            self.current_switch_states,
            self.managed_switches,
        )
        self.switch_state_publisher.publish(msg)

    def shutdown_controller(self):
        return


def main():
    parser = argparse.ArgumentParser(
        description=(
            'Subscribe to ROS 2 conveyor loop-mode topics and switch all MFJA conveyor '
            'rail blades between INTERIOR / PETIT_BOUCLE and EXTERIOR / GRAND_BOUCLE.'
        )
    )
    parser.add_argument(
        '--world',
        default='default',
        help='Gazebo world entity name used by /world/<name>/set_pose.',
    )
    parser.add_argument(
        '--world-file',
        default='',
        help='Optional world file used to load the initial switch poses.',
    )
    parser.add_argument(
        '--partition',
        default='',
        help='Gazebo transport partition to use for service calls.',
    )
    parser.add_argument(
        '--command-topic',
        default='/mfja/conveyor/loop_mode_cmd',
        help='ROS 2 std_msgs/String topic used to request INTERIOR/EXTERIOR or PETIT_BOUCLE/GRAND_BOUCLE.',
    )
    parser.add_argument(
        '--state-topic',
        default='/mfja/conveyor/loop_mode',
        help='ROS 2 std_msgs/String topic that republishes the last known applied mode.',
    )
    parser.add_argument(
        '--switch-command-topic',
        default='/mfja/conveyor/switch_cmd',
        help=(
            'ROS 2 std_msgs/String topic used to command one or more switches. '
            'Examples: "A1R=INTERIOR", "A1=EXTERIOR", '
            '"RIGHT=INTERIOR, A3L=EXTERIOR".'
        ),
    )
    parser.add_argument(
        '--switch-state-topic',
        default='/mfja/conveyor/switch_states',
        help=(
            'ROS 2 std_msgs/String topic that republishes the last known per-switch states '
            'as "switch_name=MODE" pairs.'
        ),
    )
    parser.add_argument(
        '--timeout-ms',
        type=int,
        default=1500,
        help='Timeout passed to each gz set_pose request.',
    )
    parser.add_argument(
        '--retries',
        type=int,
        default=3,
        help='How many times to retry each per-switch set_pose request.',
    )
    parser.add_argument(
        '--initial-loop-mode',
        default='auto',
        help=(
            'Loop mode to apply as soon as the controller starts. '
            'Use auto to keep the mode detected from the world file, '
            'or INTERIOR / EXTERIOR (also PETIT_BOUCLE / GRAND_BOUCLE) '
            'to force a startup mode.'
        ),
    )
    parser.add_argument(
        '--keep-paused-after-initial-loop',
        action='store_true',
        help='Leave Gazebo paused after applying --initial-loop-mode at startup.',
    )
    parser.add_argument(
        '--no-pause-during-switch-update',
        dest='pause_during_switch_update',
        action='store_false',
        help='Move visual switches without pausing the Gazebo world.',
    )
    parser.set_defaults(pause_during_switch_update=False)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ConveyorLoopModeController(args)

    try:
        rclpy.spin(node)
    finally:
        node.shutdown_controller()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
