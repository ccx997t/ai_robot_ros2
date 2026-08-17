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


def test_git_map_artifact_matches_manifest():
    manifest = yaml.safe_load(
        (MAPS / 'm5_baseline_manifest.yaml').read_text())
    metadata = yaml.safe_load((MAPS / 'm5_baseline.yaml').read_text())
    width, height, pixels = read_pgm(MAPS / 'm5_baseline.pgm')

    assert metadata['image'] == 'm5_baseline.pgm'
    assert metadata['mode'] == 'trinary'
    assert metadata['resolution'] == manifest['resolution']
    assert metadata['origin'] == manifest['origin']
    assert (width, height) == (manifest['width'], manifest['height'])
    assert len(pixels) == width * height
    assert pixels.count(0) == manifest['occupied_cells']
    assert pixels.count(254) == manifest['free_cells']
    assert pixels.count(205) == manifest['unknown_cells']
    assert sha256(MAPS / 'm5_baseline.yaml') == manifest['yaml_sha256']
    assert sha256(MAPS / 'm5_baseline.pgm') == manifest['pgm_sha256']
