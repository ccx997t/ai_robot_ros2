#!/usr/bin/env python3
"""Record reproducible S1-M5 normal and cancel/fault acceptance bags."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml


ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / 'evidence' / 'rosbags'
TOPICS = [
    '/clock',
    '/tf',
    '/tf_static',
    '/map',
    '/amcl_pose',
    '/odom',
    '/scan',
    '/cmd_vel',
    '/base_controller/cmd_vel_unstamped',
    '/diagnostics',
    '/global_costmap/costmap',
]
SCENARIOS = {
    'normal': {
        'test': 'test_test_m5_navigation_scenarios_launch.py',
        'domain': '89',
        'partition': 'ai_robot_m5_navigation_scenarios_test',
        'covers': ['repeated_navigation', 'static_obstacle',
                   'temporary_obstacle', 'unreachable_goal'],
    },
    'cancel_fault': {
        'test': 'test_test_m5_navigation_fault_recovery_launch.py',
        'domain': '90',
        'partition': 'ai_robot_m5_navigation_fault_recovery_test',
        'covers': ['navigation_cancel', 'scan_timeout',
                   'diagnostic_propagation', 'navigation_recovery'],
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def git_revision():
    return subprocess.check_output(
        ['git', 'rev-parse', '--short', 'HEAD'], cwd=ROOT,
        text=True).strip()


def summarize(directory, scenario, revision, test_returncode):
    metadata_path = directory / 'metadata.yaml'
    metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))
    info = metadata['rosbag2_bagfile_information']
    topics = {
        entry['topic_metadata']['name']: entry['message_count']
        for entry in info['topics_with_message_count']
    }
    files = {}
    for path in sorted(directory.iterdir()):
        if path.is_file():
            files[path.name] = {
                'bytes': path.stat().st_size,
                'sha256': sha256(path),
            }
    return {
        'scenario': scenario,
        'covers': SCENARIOS[scenario]['covers'],
        'git_revision': revision,
        'directory': str(directory.relative_to(ROOT)),
        'test': SCENARIOS[scenario]['test'],
        'test_returncode': test_returncode,
        'duration_ns': info['duration']['nanoseconds'],
        'message_count': info['message_count'],
        'topics': topics,
        'files': files,
        'verdict': 'PASS' if test_returncode == 0 else 'FAIL',
    }


def record(scenario, stamp, revision):
    config = SCENARIOS[scenario]
    directory = EVIDENCE / (
        f'S1-M5_{stamp}_sim_{scenario}_{revision}')
    environment = os.environ.copy()
    environment.update({
        'ROS_DOMAIN_ID': config['domain'],
        'IGN_PARTITION': config['partition'],
        'ROS_LOG_DIR': f'/tmp/ai_robot_m5_bag_{scenario}',
    })
    recorder = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(directory), *TOPICS],
        cwd=ROOT, env=environment, stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL)
    time.sleep(2.0)
    test = subprocess.run([
        'colcon', 'test', '--packages-select', 'ai_robot_bringup',
        '--ctest-args', '-R', f"^{config['test']}$", '--output-on-failure'],
        cwd=ROOT, env=environment, check=False)
    recorder.send_signal(signal.SIGINT)
    try:
        recorder.wait(timeout=15)
    except subprocess.TimeoutExpired:
        recorder.terminate()
        recorder.wait(timeout=5)
    if not (directory / 'metadata.yaml').exists():
        raise RuntimeError(f'rosbag metadata missing: {directory}')
    return summarize(directory, scenario, revision, test.returncode)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--scenario', choices=['normal', 'cancel_fault', 'all'],
        default='all')
    args = parser.parse_args()
    stamp = time.strftime('%Y%m%d-%H%M%S')
    revision = git_revision()
    EVIDENCE.mkdir(parents=True, exist_ok=True)
    names = list(SCENARIOS) if args.scenario == 'all' else [args.scenario]
    summaries = [record(name, stamp, revision) for name in names]
    summary_path = EVIDENCE / f'S1-M5_{stamp}_summary.json'
    summary_path.write_text(
        json.dumps({'samples': summaries}, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    print(json.dumps({'summary': str(summary_path), 'samples': summaries},
                     indent=2, sort_keys=True))
    raise SystemExit(0 if all(item['verdict'] == 'PASS'
                              for item in summaries) else 1)


if __name__ == '__main__':
    main()
