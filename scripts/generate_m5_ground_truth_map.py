#!/usr/bin/env python3
"""Generate the deterministic M5 simulation ground-truth occupancy map."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / 'src' / 'ai_robot_sim' / 'config' / 'm5_scenario.yaml'
MAP_DIR = ROOT / 'src' / 'ai_robot_bringup' / 'maps'
PREFIX = 'm5_complete'
RESOLUTION = 0.05
PADDING = 0.10


def inside_box(x: float, y: float, pose: list[float], size: list[float]) -> bool:
    center_x, center_y, _, yaw = pose
    local_x = math.cos(yaw) * (x - center_x) + math.sin(yaw) * (y - center_y)
    local_y = -math.sin(yaw) * (x - center_x) + math.cos(yaw) * (y - center_y)
    return abs(local_x) <= size[0] / 2.0 and abs(local_y) <= size[1] / 2.0


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    scenario = yaml.safe_load(SCENARIO.read_text(encoding='utf-8'))['scenario']
    boundary = scenario['boundary']
    origin_x = float(boundary['x_min']) - PADDING
    origin_y = float(boundary['y_min']) - PADDING
    maximum_x = float(boundary['x_max']) + PADDING
    maximum_y = float(boundary['y_max']) + PADDING
    width = round((maximum_x - origin_x) / RESOLUTION)
    height = round((maximum_y - origin_y) / RESOLUTION)
    models = list(scenario['static_models'].values())

    rows: list[bytes] = []
    counts = {0: 0, 205: 0, 254: 0}
    # PGM rows run from maximum Y to minimum Y, while OccupancyGrid uses the
    # lower-left origin declared in the YAML metadata.
    for image_row in range(height):
        y = origin_y + (height - image_row - 0.5) * RESOLUTION
        row = bytearray()
        for column in range(width):
            x = origin_x + (column + 0.5) * RESOLUTION
            if any(inside_box(x, y, model['pose'], model['size'])
                   for model in models):
                pixel = 0
            elif (boundary['x_min'] < x < boundary['x_max']
                  and boundary['y_min'] < y < boundary['y_max']):
                pixel = 254
            else:
                pixel = 205
            row.append(pixel)
            counts[pixel] += 1
        rows.append(bytes(row))

    pgm = MAP_DIR / f'{PREFIX}.pgm'
    metadata = MAP_DIR / f'{PREFIX}.yaml'
    manifest = MAP_DIR / f'{PREFIX}_manifest.yaml'
    pgm.write_bytes(
        f'P5\n# deterministic M5 simulation ground-truth map\n'
        f'{width} {height}\n255\n'.encode('ascii') + b''.join(rows))
    metadata.write_text(
        f'image: {PREFIX}.pgm\n'
        'mode: trinary\n'
        f'resolution: {RESOLUTION}\n'
        f'origin: [{origin_x}, {origin_y}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.19\n',
        encoding='utf-8')
    manifest_data = {
        'artifact': PREFIX,
        'kind': 'simulation_ground_truth',
        'source': 'ai_robot_sim/config/m5_scenario.yaml',
        'format': 'nav2_trinary_pgm',
        'width': width,
        'height': height,
        'resolution': RESOLUTION,
        'origin': [origin_x, origin_y, 0.0],
        'occupied_cells': counts[0],
        'free_cells': counts[254],
        'unknown_cells': counts[205],
        'yaml_sha256': sha256(metadata),
        'pgm_sha256': sha256(pgm),
    }
    manifest.write_text(
        yaml.safe_dump(manifest_data, sort_keys=False), encoding='utf-8')


if __name__ == '__main__':
    main()
