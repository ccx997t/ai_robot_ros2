#!/usr/bin/env python3
"""Measure and enforce the S1-M5 complete-navigation resource baseline."""

import argparse
from collections import defaultdict, deque
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


THRESHOLDS = {
    'cpu_percent_p95_max': 800.0,
    'rss_mib_p95_max': 1800.0,
    'frequency_hz': {
        '/odom': [24.0, 36.0],
        '/scan': [8.0, 12.0],
        '/global_costmap/costmap': [0.6, 1.3],
    },
    'freshness_p95_ms_max': {
        '/odom': 60.0,
        '/scan': 60.0,
        '/global_costmap/costmap': 1500.0,
    },
    'cmd_vel_to_controller_p95_ms_max': 50.0,
    'map_width': 244,
    'map_height': 204,
}


def percentile(values, ratio):
    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[min(len(ordered) - 1, int(len(ordered) * ratio))]


def descendants(root_pid):
    children = defaultdict(list)
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / 'stat').read_text().split()
            children[int(fields[3])].append(int(entry.name))
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    result = set()
    pending = [root_pid]
    while pending:
        parent = pending.pop()
        for child in children.get(parent, []):
            if child not in result:
                result.add(child)
                pending.append(child)
    return result


def process_totals(pids):
    ticks = 0
    rss_pages = 0
    for pid in pids:
        try:
            fields = Path(f'/proc/{pid}/stat').read_text().split()
            ticks += int(fields[13]) + int(fields[14])
            rss_pages += int(fields[23])
        except (FileNotFoundError, PermissionError, ValueError):
            continue
    return ticks, rss_pages * os.sysconf('SC_PAGE_SIZE')


class MetricsNode:
    def __init__(self, node):
        self.node = node
        self.clock_ns = 0
        self.arrivals = defaultdict(lambda: deque(maxlen=5000))
        self.freshness_ms = defaultdict(lambda: deque(maxlen=5000))
        self.command_sent = deque()
        self.command_latency_ms = deque(maxlen=1000)
        self.maps = deque(maxlen=2)
        self.command_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        map_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subscriptions = [
            node.create_subscription(
                Clock, '/clock', self.clock, qos_profile_sensor_data),
            node.create_subscription(
                Odometry, '/odom', lambda m: self.message('/odom', m), 10),
            node.create_subscription(
                LaserScan, '/scan', lambda m: self.message('/scan', m),
                qos_profile_sensor_data),
            node.create_subscription(
                PoseWithCovarianceStamped, '/amcl_pose',
                lambda m: self.message('/amcl_pose', m), 10),
            node.create_subscription(
                OccupancyGrid, '/global_costmap/costmap',
                lambda m: self.message('/global_costmap/costmap', m),
                map_qos),
            node.create_subscription(
                OccupancyGrid, '/map', self.maps.append, map_qos),
            node.create_subscription(
                Twist, '/base_controller/cmd_vel_unstamped',
                self.controller_command, 10),
        ]

    @staticmethod
    def stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def clock(self, message):
        self.clock_ns = message.clock.sec * 1_000_000_000 + message.clock.nanosec

    def message(self, topic, message):
        self.arrivals[topic].append(time.monotonic())
        stamp = self.stamp_ns(message)
        if self.clock_ns and 0 <= self.clock_ns - stamp < 2_000_000_000:
            self.freshness_ms[topic].append((self.clock_ns - stamp) / 1e6)

    def publish_probe(self):
        self.command_sent.append(time.monotonic())
        self.command_pub.publish(Twist())

    def controller_command(self, message):
        if self.command_sent and message.linear.x == 0.0 and message.angular.z == 0.0:
            self.command_latency_ms.append(
                (time.monotonic() - self.command_sent.popleft()) * 1000.0)


def evaluate(result):
    failures = []
    resources = result['process_tree']
    if resources['cpu_percent_p95'] > THRESHOLDS['cpu_percent_p95_max']:
        failures.append('process_tree.cpu_percent_p95')
    if resources['rss_mib_p95'] > THRESHOLDS['rss_mib_p95_max']:
        failures.append('process_tree.rss_mib_p95')
    for topic, limits in THRESHOLDS['frequency_hz'].items():
        value = result['frequency_hz'].get(topic)
        if value is None or not limits[0] <= value <= limits[1]:
            failures.append(f'frequency_hz.{topic}')
    for topic, maximum in THRESHOLDS['freshness_p95_ms_max'].items():
        value = result['freshness_p95_ms'].get(topic)
        if value is None or value > maximum:
            failures.append(f'freshness_p95_ms.{topic}')
    latency = result['latency_p95_ms']['cmd_vel_to_controller']
    if latency is None or latency > THRESHOLDS['cmd_vel_to_controller_p95_ms_max']:
        failures.append('latency_p95_ms.cmd_vel_to_controller')
    if result['map_geometry'] != {
            'width': THRESHOLDS['map_width'],
            'height': THRESHOLDS['map_height']}:
        failures.append('map_geometry')
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--warmup', type=float, default=20.0)
    parser.add_argument('--duration', type=float, default=30.0)
    args = parser.parse_args()
    launch = subprocess.Popen([
        'ros2', 'launch', 'ai_robot_bringup', 'navigation.launch.py',
        'mode:=sim'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node('m5_resource_measurement')
    metrics = MetricsNode(node)
    cpu_samples = []
    rss_samples = []
    previous = None
    next_sample = time.monotonic() + args.warmup
    end = next_sample + args.duration
    next_probe = next_sample
    ticks_per_second = os.sysconf(os.sysconf_names['SC_CLK_TCK'])
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_probe:
                metrics.publish_probe()
                next_probe += 0.25
            if now >= next_sample:
                totals = process_totals(descendants(launch.pid) | {launch.pid})
                if previous is not None:
                    elapsed = now - previous[0]
                    cpu_samples.append(
                        (totals[0] - previous[1]) / ticks_per_second
                        / elapsed * 100.0)
                previous = (now, totals[0])
                rss_samples.append(totals[1] / 1024 / 1024)
                next_sample += 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        launch.send_signal(signal.SIGINT)
        try:
            launch.wait(timeout=12)
        except subprocess.TimeoutExpired:
            launch.terminate()
            launch.wait(timeout=5)

    frequencies = {}
    for topic, arrivals in metrics.arrivals.items():
        samples = [value for value in arrivals if value >= end - args.duration]
        if len(samples) >= 2:
            frequencies[topic] = (len(samples) - 1) / (samples[-1] - samples[0])
    latest_map = metrics.maps[-1] if metrics.maps else None
    result = {
        'measurement_seconds': args.duration,
        'process_tree': {
            'cpu_percent_p95': percentile(cpu_samples, 0.95),
            'cpu_percent_max': max(cpu_samples, default=None),
            'rss_mib_p95': percentile(rss_samples, 0.95),
            'rss_mib_max': max(rss_samples, default=None),
        },
        'frequency_hz': frequencies,
        'freshness_p95_ms': {
            topic: percentile(values, 0.95)
            for topic, values in metrics.freshness_ms.items()},
        'latency_p95_ms': {
            'cmd_vel_to_controller': percentile(
                metrics.command_latency_ms, 0.95)},
        'map_geometry': ({
            'width': latest_map.info.width,
            'height': latest_map.info.height,
        } if latest_map else None),
        'sample_counts': {
            topic: len(values) for topic, values in metrics.arrivals.items()},
        'thresholds': THRESHOLDS,
    }
    failures = evaluate(result)
    result['verdict'] = 'PASS' if not failures else 'FAIL'
    result['failures'] = failures
    print(json.dumps(result, indent=2, sort_keys=True))
    raise SystemExit(0 if not failures else 1)


if __name__ == '__main__':
    main()
