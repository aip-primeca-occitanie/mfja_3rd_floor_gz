#!/usr/bin/env python3

import argparse

import rclpy
from mfja_rail_interfaces.msg import SensorFeedback


def _default_topics(side: str) -> list[str]:
    return [f'/room_315/rails/{side}/sensors/feedback']


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Watch one Room 315 rail sensor by name.'
    )
    parser.add_argument(
        '--side',
        choices=('right', 'left'),
        default='right',
        help='Rail side to watch. Default: right.',
    )
    parser.add_argument(
        '--name',
        required=True,
        help='Sensor name, for example A1_APPROACH, DZI2R, or DZI2L.',
    )
    parser.add_argument(
        '--topic',
        default='',
        help='Override the auto-selected sensor feedback topic.',
    )
    parser.add_argument(
        '--once',
        action='store_true',
        help='Print the first reading for this sensor and exit.',
    )
    parser.add_argument(
        '--all-updates',
        action='store_true',
        help='Print every message instead of only changes.',
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    sensor_name = args.name.strip().upper()
    sensor_topics = (
        [args.topic.strip()]
        if args.topic.strip()
        else _default_topics(args.side)
    )
    last_reading = None
    missing_reported = False
    found_once = False
    seen_topics: set[str] = set()

    rclpy.init()
    node = rclpy.create_node('single_sensor_feedback_watcher')

    def on_feedback(message: SensorFeedback, sensor_topic: str) -> None:
        nonlocal last_reading
        nonlocal missing_reported
        nonlocal found_once

        seen_topics.add(sensor_topic)

        for reading in message.readings:
            if reading.name.upper() != sensor_name:
                continue

            missing_reported = False
            found_once = True
            current = (
                int(reading.active),
                reading.shuttle_name,
                reading.segment,
                round(reading.s, 4),
                round(reading.s_ratio, 4),
            )
            if args.all_updates or current != last_reading:
                last_reading = current
                shuttle = reading.shuttle_name or '-'
                state = 'occupied' if reading.active else 'clear'
                print(
                    f'{reading.name}: active={int(reading.active)} ({state}) '
                    f'shuttle={shuttle} segment={reading.segment} '
                    f's={reading.s:.3f} s_ratio={reading.s_ratio:.3f}',
                    flush=True,
                )

            if args.once:
                raise KeyboardInterrupt
            return

        if not found_once and len(seen_topics) == len(sensor_topics) and not missing_reported:
            missing_reported = True
            topics = ', '.join(sensor_topics)
            print(f'{sensor_name}: not present on watched sensor topics: {topics}', flush=True)

    for sensor_topic in sensor_topics:
        node.create_subscription(
            SensorFeedback,
            sensor_topic,
            lambda message, topic=sensor_topic: on_feedback(message, topic),
            10,
        )
    print(
        f'Watching {sensor_name} on {", ".join(sensor_topics)}. Press Ctrl-C to stop.',
        flush=True,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
