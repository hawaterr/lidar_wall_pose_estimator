#!/usr/bin/env python3
"""Generates the practice rosbag (synthetic 2D LiDAR + IMU data).

The bag is written directly in the ROS 2 Humble format (sqlite3, metadata version 5, schema 3),
which newer distros (Iron, Jazzy) can still read. rosbags is only used to serialize the messages:
its Writer only produces the newer formats that Humble can't read.

Scenario: an inspection drone approaches a flat wall (e.g. a kiln shell), oscillates in yaw,
holds close to the surface, turns away so no wall is in front, comes back, and flies through
a dust cloud that produces spurious short returns.

Requires:  pip install rosbags numpy
Usage:     python3 generate_bag.py <output_bag_dir>
"""
import math
import sqlite3
import sys
from pathlib import Path

import numpy as np
from rosbags.typesys import Stores, get_typestore

RNG = np.random.default_rng(42)

SCAN_RATE = 20.0          # Hz
IMU_RATE = 100.0          # Hz
DURATION = 60.0           # s
ANGLE_MIN = math.radians(-135.0)
ANGLE_MAX = math.radians(135.0)
ANGLE_INC = math.radians(0.5)
RANGE_MIN = 0.10
RANGE_MAX = 10.0
NOISE_STD = 0.01          # m
WALL_HALF_LENGTH = 4.0    # m, the wall is a finite segment
DROPOUT_PROB = 0.03       # fraction of rays returning NaN
START_NS = 1_760_000_000 * 10**9

QOS = (
    "- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n"
    "  deadline:\n    sec: 2147483647\n    nsec: 4294967295\n"
    "  lifespan:\n    sec: 2147483647\n    nsec: 4294967295\n"
    "  liveliness: 1\n"
    "  liveliness_lease_duration:\n    sec: 2147483647\n    nsec: 4294967295\n"
    "  avoid_ros_namespace_conventions: false"
)


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


def lerp(a, b, x):
    return a + (b - a) * smoothstep(x)


def scenario(t):
    """Returns (distance_to_wall [m], wall normal angle in drone frame [rad], dust_prob)."""
    if t < 10:      # hover in front of the wall
        d, yaw = 3.0, 0.0
    elif t < 30:    # approach while oscillating in yaw
        d = lerp(3.0, 1.0, (t - 10) / 20)
        yaw = math.radians(20.0) * math.sin(2 * math.pi * (t - 10) / 8.0)
    elif t < 38:    # slow approach to contact distance
        d = lerp(1.0, 0.45, (t - 30) / 8)
        yaw = math.radians(5.0) * math.sin(2 * math.pi * (t - 30) / 4.0)
    elif t < 47:    # back off and turn away from the wall, then come back
        d = lerp(0.45, 1.5, (t - 38) / 2)
        if t < 40:
            yaw = 0.0
        elif t < 42:
            yaw = lerp(0.0, math.radians(160.0), (t - 40) / 2)
        elif t < 45:
            yaw = math.radians(160.0)
        else:
            yaw = lerp(math.radians(160.0), math.radians(-10.0), (t - 45) / 2)
    else:           # hold near the wall while flying through dust
        d = lerp(1.5, 1.2, (t - 47) / 5)
        yaw = math.radians(-10.0) + math.radians(8.0) * math.sin(2 * math.pi * (t - 47) / 6.0)
    dust = 0.12 if 50 <= t < 56 else 0.0
    return d, yaw, dust


def wall_range(theta, d, psi):
    """Range along a ray at angle theta to a wall with normal angle psi at distance d."""
    c = math.cos(theta - psi)
    if c <= 1e-3:
        return math.inf
    r = d / c
    # position along the wall relative to the foot of the perpendicular
    along = d * math.tan(theta - psi)
    if abs(along) > WALL_HALF_LENGTH:
        return math.inf
    return r


def pillar_range(theta, t):
    """A pipe on the left side of the drone (outside the front field of view)."""
    center_angle = math.radians(70.0)
    center_dist = 1.2
    radius = 0.15
    # ray-circle intersection
    cx, cy = center_dist * math.cos(center_angle), center_dist * math.sin(center_angle)
    dx, dy = math.cos(theta), math.sin(theta)
    b = dx * cx + dy * cy
    disc = b * b - (cx * cx + cy * cy - radius * radius)
    if disc < 0:
        return math.inf
    r = b - math.sqrt(disc)
    return r if r > 0 else math.inf


def make_scan(t):
    d, psi, dust = scenario(t)
    n = int(round((ANGLE_MAX - ANGLE_MIN) / ANGLE_INC)) + 1
    ranges = np.empty(n, dtype=np.float32)
    for i in range(n):
        theta = ANGLE_MIN + i * ANGLE_INC
        r = min(wall_range(theta, d, psi), pillar_range(theta, t))
        if math.isfinite(r):
            r += RNG.normal(0.0, NOISE_STD)
        if dust > 0 and RNG.random() < dust:
            r = RNG.uniform(0.15, 0.8 * min(r, 3.0)) if math.isfinite(r) else RNG.uniform(0.2, 2.0)
        if RNG.random() < DROPOUT_PROB:
            r = math.nan
        elif math.isfinite(r) and (r < RANGE_MIN or r > RANGE_MAX):
            r = math.inf
        ranges[i] = r
    return ranges


def write_humble_bag(out_bag, topics, messages):
    """Writes a rosbag2 bag in the Humble format.

    topics: list of (topic_id, name, type); messages: list of (timestamp_ns, topic_id, cdr_bytes).
    """
    out_bag.mkdir(parents=True, exist_ok=False)
    name = out_bag.name + '_0.db3'
    messages = sorted(messages, key=lambda m: m[0])

    db = sqlite3.connect(out_bag / name)
    db.executescript("""
        CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
        INSERT INTO schema VALUES (3, 'humble');
        CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,
                            serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);
        CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,
                              timestamp INTEGER NOT NULL, data BLOB NOT NULL);
        CREATE INDEX timestamp_idx ON messages (timestamp ASC);
    """)
    db.executemany('INSERT INTO topics VALUES (?,?,?,?,?)',
                   [(tid, tname, ttype, 'cdr', QOS) for tid, tname, ttype in topics])
    db.executemany('INSERT INTO messages VALUES (?,?,?,?)',
                   [(i + 1, tid, stamp, data) for i, (stamp, tid, data) in enumerate(messages)])
    db.commit()
    db.close()

    start, end, total = messages[0][0], messages[-1][0], len(messages)
    counts = {}
    for _, tid, _ in messages:
        counts[tid] = counts.get(tid, 0) + 1

    qos_yaml = QOS.replace('\n', '\\n')
    lines = [
        'rosbag2_bagfile_information:',
        '  version: 5',
        '  storage_identifier: sqlite3',
        '  duration:',
        f'    nanoseconds: {end - start}',
        '  starting_time:',
        f'    nanoseconds_since_epoch: {start}',
        f'  message_count: {total}',
        '  topics_with_message_count:',
    ]
    for tid, tname, ttype in topics:
        lines += [
            '    - topic_metadata:',
            f'        name: {tname}',
            f'        type: {ttype}',
            '        serialization_format: cdr',
            f'        offered_qos_profiles: "{qos_yaml}"',
            f'      message_count: {counts.get(tid, 0)}',
        ]
    lines += [
        '  compression_format: ""',
        '  compression_mode: ""',
        '  relative_file_paths:',
        f'    - {name}',
        '  files:',
        f'    - path: {name}',
        '      starting_time:',
        f'        nanoseconds_since_epoch: {start}',
        '      duration:',
        f'        nanoseconds: {end - start}',
        f'      message_count: {total}',
    ]
    (out_bag / 'metadata.yaml').write_text('\n'.join(lines) + '\n')


def main():
    out_bag = Path(sys.argv[1])
    ts = get_typestore(Stores.ROS2_HUMBLE)
    LaserScan = ts.types['sensor_msgs/msg/LaserScan']
    Imu = ts.types['sensor_msgs/msg/Imu']
    Header = ts.types['std_msgs/msg/Header']
    Time = ts.types['builtin_interfaces/msg/Time']
    Quaternion = ts.types['geometry_msgs/msg/Quaternion']
    Vector3 = ts.types['geometry_msgs/msg/Vector3']

    def stamp(ns):
        return Time(sec=ns // 10**9, nanosec=ns % 10**9)

    SCAN_TOPIC, IMU_TOPIC = 1, 2
    topics = [(SCAN_TOPIC, '/scan', LaserScan.__msgtype__),
              (IMU_TOPIC, '/imu/data', Imu.__msgtype__)]
    messages = []
    events = []
    for k in range(int(DURATION * SCAN_RATE)):
        events.append((k / SCAN_RATE, 'scan'))
    for k in range(int(DURATION * IMU_RATE)):
        events.append((k / IMU_RATE, 'imu'))
    events.sort(key=lambda e: (e[0], e[1]))

    prev_yaw = 0.0
    for t, kind in events:
        # header stamp = acquisition time; bag time = arrival time (a few ms later)
        stamp_ns = START_NS + int(t * 1e9)
        if kind == 'scan':
            ranges = make_scan(t)
            msg = LaserScan(
                header=Header(stamp=stamp(stamp_ns), frame_id='lidar_link'),
                angle_min=ANGLE_MIN, angle_max=ANGLE_MAX, angle_increment=ANGLE_INC,
                time_increment=0.0, scan_time=1.0 / SCAN_RATE,
                range_min=RANGE_MIN, range_max=RANGE_MAX,
                ranges=ranges, intensities=np.zeros(0, dtype=np.float32))
            recv_ns = stamp_ns + int(RNG.uniform(8e6, 25e6))
            messages.append((recv_ns, SCAN_TOPIC, ts.serialize_cdr(msg, LaserScan.__msgtype__)))
        else:
            _, psi, _ = scenario(t)
            yaw = -psi  # drone yaw in world is the opposite of the wall angle in body frame
            yaw_rate = (yaw - prev_yaw) * IMU_RATE
            prev_yaw = yaw
            msg = Imu(
                header=Header(stamp=stamp(stamp_ns), frame_id='imu_link'),
                orientation=Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
                orientation_covariance=np.zeros(9),
                angular_velocity=Vector3(x=RNG.normal(0, 0.01), y=RNG.normal(0, 0.01),
                                         z=yaw_rate + RNG.normal(0, 0.01)),
                angular_velocity_covariance=np.zeros(9),
                linear_acceleration=Vector3(x=RNG.normal(0, 0.05), y=RNG.normal(0, 0.05),
                                            z=9.81 + RNG.normal(0, 0.05)),
                linear_acceleration_covariance=np.zeros(9))
            recv_ns = stamp_ns + int(RNG.uniform(1e6, 3e6))
            messages.append((recv_ns, IMU_TOPIC, ts.serialize_cdr(msg, Imu.__msgtype__)))

    write_humble_bag(out_bag, topics, messages)


if __name__ == '__main__':
    main()
