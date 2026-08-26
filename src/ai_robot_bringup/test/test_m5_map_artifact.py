import hashlib
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
MAPS = ROOT / 'maps'


def read_pgm(path):
    content = path.read_bytes()
    tokens = []
    index = 0
    while len(tokens) < 4:
        while content[index:index + 1].isspace():
            index += 1
        if content[index:index + 1] == b'#':
            index = content.index(b'\n', index) + 1
            continue
        end = index
        while not content[end:end + 1].isspace():
            end += 1
        tokens.append(content[index:end])
        index = end
    while content[index:index + 1].isspace():
        index += 1
    return int(tokens[1]), int(tokens[2]), content[index:]


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_artifact(name):
    manifest = yaml.safe_load((MAPS / f'{name}_manifest.yaml').read_text())
    metadata = yaml.safe_load((MAPS / f'{name}.yaml').read_text())
    width, height, pixels = read_pgm(MAPS / f'{name}.pgm')

    assert metadata['image'] == f'{name}.pgm'
    assert metadata['mode'] == 'trinary'
    assert metadata['resolution'] == manifest['resolution']
    assert metadata['origin'] == manifest['origin']
    assert (width, height) == (manifest['width'], manifest['height'])
    assert len(pixels) == width * height
    assert pixels.count(0) == manifest['occupied_cells']
    assert pixels.count(254) == manifest['free_cells']
    assert pixels.count(205) == manifest['unknown_cells']
    assert sha256(MAPS / f'{name}.yaml') == manifest['yaml_sha256']
    assert sha256(MAPS / f'{name}.pgm') == manifest['pgm_sha256']
    return metadata, manifest, width, height, pixels


def test_git_map_artifact_matches_manifest():
    validate_artifact('m5_baseline')


def test_complete_map_matches_frozen_scenario():
    metadata, manifest, width, height, pixels = validate_artifact(
        'm5_complete')
    scenario_path = (
        ROOT.parents[0] / 'ai_robot_sim' / 'config' / 'm5_scenario.yaml')
    scenario = yaml.safe_load(scenario_path.read_text())['scenario']

    assert manifest['kind'] == 'simulation_ground_truth'
    assert manifest['source'] == 'ai_robot_sim/config/m5_scenario.yaml'
    assert manifest['unknown_cells'] == 0
    assert manifest['free_cells'] > 40000
    assert manifest['occupied_cells'] > 5000

    origin_x, origin_y, _ = metadata['origin']
    resolution = metadata['resolution']

    def pixel_at(x, y):
        column = int((x - origin_x) / resolution)
        row_from_bottom = int((y - origin_y) / resolution)
        image_row = height - row_from_bottom - 1
        return pixels[image_row * width + column]

    for model in scenario['static_models'].values():
        assert pixel_at(model['pose'][0], model['pose'][1]) == 0
    for goal in scenario['goals'].values():
        assert pixel_at(goal['x'], goal['y']) == 254
