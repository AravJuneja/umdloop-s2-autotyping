"""Turn a recorded rosbag into a line-per-message JSON evidence log.

Reads a bag directory (sqlite3) and writes one JSON object per message::

    {"t": <stamp_ns>, "topic": ..., "type": ..., "msg": {...}}

Message fields are decoded with the ROS message classes, so the custom
interfaces (autotype_msgs, interfaces) must be importable when this runs --
i.e. inside the dev container, not on the host.

Usage: python3 scripts/bag_to_jsonl.py BAG_DIR [-o OUT.jsonl]
"""

import argparse
import json
import os
import re
import sys


def _json_default(value):
    """Fall back for values json.dump cannot handle on its own."""
    if isinstance(value, (bytes, bytearray)):
        return value.hex()
    tolist = getattr(value, 'tolist', None)
    if tolist is not None:
        return tolist()
    return str(value)


def _msg_class(type_str, cache):
    if not type_str:
        return None
    if type_str not in cache:
        from rosidl_runtime_py.utilities import get_message

        cache[type_str] = get_message(type_str)
    return cache[type_str]


def storage_id(bag_dir, override=''):
    """The bag's storage plugin: ``override`` if given, else from metadata.yaml.

    ``ros2 bag record`` defaults to mcap in Jazzy, not sqlite3, so guessing is
    not safe -- read what the recorder wrote.
    """
    if override:
        return override
    try:
        with open(os.path.join(bag_dir, 'metadata.yaml')) as handle:
            for line in handle:
                match = re.match(r'\s*storage_identifier:\s*(\S+)', line)
                if match:
                    return match.group(1)
    except OSError:
        pass
    return 'mcap'


def convert(bag_dir, out, storage=''):
    """Write every message in ``bag_dir`` to ``out`` as one JSON line each."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.convert import message_to_ordereddict

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id=storage_id(bag_dir, storage)),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    cache = {}
    count = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        type_str = types.get(topic, '')
        record = {'t': stamp, 'topic': topic, 'type': type_str}
        message_type = _msg_class(type_str, cache)
        if message_type is None:
            record['decode_error'] = f'unknown type for {topic!r}'
        else:
            try:
                record['msg'] = message_to_ordereddict(deserialize_message(data, message_type))
            except Exception as exc:  # noqa: BLE001 -- keep the line, note the failure
                record['decode_error'] = str(exc)
        json.dump(record, out, default=_json_default)
        out.write('\n')
        count += 1
    return count


def main(argv=None):
    parser = argparse.ArgumentParser(description='Convert a rosbag to JSONL evidence.')
    parser.add_argument('bag', help='bag directory to read')
    parser.add_argument('-o', '--out', default='', help='output .jsonl (default: stdout)')
    parser.add_argument(
        '--storage-id',
        default='',
        help='rosbag2 storage plugin (default: read metadata.yaml)',
    )
    args = parser.parse_args(argv)

    handle = open(args.out, 'w') if args.out else sys.stdout
    try:
        count = convert(args.bag, handle, args.storage_id)
    finally:
        if args.out:
            handle.close()
    print(f'{count} messages -> {args.out or "stdout"}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())