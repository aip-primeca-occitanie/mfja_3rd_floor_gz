#!/usr/bin/env python3

import argparse
import math
import os
import subprocess
import sys
import time


SWITCH_POSITIONS = {
    '0': 2.6,
    '1': -1.59,
    '2': 0.50666,
}

def quaternion_from_yaw(yaw: float):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def main():
    parser = argparse.ArgumentParser(
        description='Instantly set the three-position rail switch to one of the calibrated yaw angles.'
    )
    parser.add_argument(
        'position',
        choices=sorted(SWITCH_POSITIONS),
        help='Discrete switch position: 0, 1, or 2',
    )
    parser.add_argument(
        '--world',
        default='default',
        help='Gazebo world name. Use "default" for the full floor and "room_315_only" for the room-only world.',
    )
    parser.add_argument(
        '--entity',
        default='rail_switch_3pos_left',
        help='Gazebo model name of the moving switch blade.',
    )
    parser.add_argument(
        '--pos',
        nargs=3,
        type=float,
        default=[-16.73, -4.42, 0.73],
        metavar=('X', 'Y', 'Z'),
        help='World position of the switch pivot model origin.',
    )
    parser.add_argument(
        '--timeout-ms',
        type=int,
        default=1500,
        help='Timeout passed to gz service.',
    )
    parser.add_argument(
        '--retries',
        type=int,
        default=3,
        help='How many times to retry the gz set_pose service before failing.',
    )
    parser.add_argument(
        '--partition',
        default=os.environ.get('GZ_PARTITION', 'rail_switch_demo'),
        help='Gazebo transport partition. Defaults to $GZ_PARTITION or rail_switch_demo.',
    )
    args = parser.parse_args()

    yaw = SWITCH_POSITIONS[args.position]
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    x, y, z = args.pos

    req = (
        f'name: "{args.entity}", '
        f'position: {{x: {x}, y: {y}, z: {z}}}, '
        f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}'
    )
    cmd = [
        'gz', 'service',
        '-s', f'/world/{args.world}/set_pose',
        '--reqtype', 'gz.msgs.Pose',
        '--reptype', 'gz.msgs.Boolean',
        '--timeout', str(args.timeout_ms),
        '--req', req,
    ]

    print(
        f'Setting {args.entity} in world {args.world} to position {args.position} '
        f'({yaw:.3f} rad) at pose ({x}, {y}, {z}) on partition {args.partition}.'
    )
    env = os.environ.copy()
    env['GZ_PARTITION'] = args.partition
    last_output = ''
    for attempt in range(1, args.retries + 1):
        completed = subprocess.run(
            cmd,
            check=False,
            env=env,
            text=True,
            capture_output=True,
        )
        output = '\n'.join(part for part in [completed.stdout, completed.stderr] if part).strip()
        last_output = output
        if output:
            print(output)

        timed_out = 'timed out' in output.lower()
        success = completed.returncode == 0 and not timed_out
        if success:
            print(f'Done on attempt {attempt}.')
            return

        if attempt < args.retries:
            print(f'Attempt {attempt} failed, retrying...')
            time.sleep(0.25)

    raise SystemExit(
        f'Failed to set switch pose in world {args.world} on partition {args.partition}. '
        f'Last output: {last_output}'
    )


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as exc:
        print(f'Failed to set switch pose: {exc}', file=sys.stderr)
        raise SystemExit(exc.returncode)
