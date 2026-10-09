"""Validate the shipped browser meshes without Blender or the sibling CAD repo."""
import hashlib
import json
import math
from pathlib import Path
import struct

import pytest

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'ros_ws/gui/assets/robots'


@pytest.fixture(scope='module')
def asset():
    data = (ASSETS / 'robot-parts.glb').read_bytes()
    magic, version, length = struct.unpack_from('<4sII', data)
    assert (magic, version, length) == (b'glTF', 2, len(data))
    json_length, kind = struct.unpack_from('<I4s', data, 12)
    assert kind == b'JSON'
    gltf = json.loads(data[20:20 + json_length])
    binary_length, kind = struct.unpack_from('<I4s', data, 20 + json_length)
    assert kind == b'BIN\0'
    binary = data[28 + json_length:]
    assert len(binary) == binary_length
    return gltf, binary, len(data)


def values(gltf, binary, accessor_index):
    accessor = gltf['accessors'][accessor_index]
    view = gltf['bufferViews'][accessor['bufferView']]
    fmt = {5121: 'B', 5123: 'H', 5125: 'I', 5126: 'f'}[accessor['componentType']]
    width = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4}[accessor['type']]
    unpacker = struct.Struct('<' + fmt * width)
    offset = view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
    stride = view.get('byteStride', unpacker.size)
    return [unpacker.unpack_from(binary, offset + i * stride) for i in range(accessor['count'])]


def test_self_contained_and_bounded(asset):
    gltf, binary, size = asset
    manifest = json.loads((ASSETS / 'manifest.json').read_text())
    assert size == manifest['bytes'] < 22_000_000
    assert not gltf.get('images')  # No textures, decoder or external asset requests.
    assert all('uri' not in b for b in gltf['buffers'])
    assert not gltf.get('extensionsRequired')
    assert {n['name'] for n in gltf['nodes']} == set(manifest['parts'])
    scene_triangles = 0
    for node in gltf['nodes']:
        # Units/offsets are baked once; each runtime object has a rigid transform.
        assert not any(k in node for k in ('scale', 'rotation', 'translation', 'matrix'))
        primitives = gltf['meshes'][node['mesh']]['primitives']
        assert len(primitives) == 1  # One draw call per rigid mesh instance.
        primitive = primitives[0]
        points = values(gltf, binary, primitive['attributes']['POSITION'])
        assert all(math.isfinite(v) for point in points for v in point)
        assert max(abs(v) for point in points for v in point) < 1
        indices = values(gltf, binary, primitive['indices'])
        assert len(indices) % 3 == 0
        assert all(0 <= row[0] < len(points) for row in indices)
        triangles = len(indices) // 3
        part = manifest['parts'][node['name']]
        assert triangles == part['triangles']
        assert 0 <= part['sampled_max_error_mm'] <= manifest['tolerance_mm'] <= 1
        assert 'COLOR_0' in primitive['attributes']
        colors = values(gltf, binary, primitive['attributes']['COLOR_0'])
        assert len(set(colors)) > 1, 'CAD material colours were lost'
        scene_triangles += triangles * (6 if node['name'] in ('jb_inner', 'jb_outer') else 1)
    assert scene_triangles == manifest['scene_triangles'] < 500_000


def test_assets_match_source_cad_when_available():
    manifest = json.loads((ASSETS / 'manifest.json').read_text())
    source_dir = ROOT / 'temp/gui-robot-source'
    if not source_dir.exists():
        pytest.skip('Original CAD exports are kept offline, outside the deployed GUI')
    for name, part in manifest['parts'].items():
        source = source_dir / part['source']
        assert hashlib.sha256(source.read_bytes()).hexdigest() == part['sha256'], 'Rebuild GUI CAD assets'


def test_joint_local_coordinates(asset):
    gltf, binary, _ = asset
    bounds = {}
    for node in gltf['nodes']:
        primitive = gltf['meshes'][node['mesh']]['primitives'][0]
        points = values(gltf, binary, primitive['attributes']['POSITION'])
        bounds[node['name']] = ([min(p[i] for p in points) for i in range(3)],
                                [max(p[i] for p in points) for i in range(3)])
    # glTF Y is robot Z. Pin physical offsets, not just relative articulation:
    # a mistaken mm/metre conversion can otherwise pass a moving-joint test.
    assert bounds['jb_base'][0][1] == pytest.approx(-.082, abs=.003)
    assert bounds['jb_outer'][1][1] == pytest.approx(.609, abs=.005)
    assert bounds['jb_inner'][1][1] == pytest.approx(.013, abs=.003)
    assert bounds['jb_inner'][0][1] == pytest.approx(-.435, abs=.003)
    # Owner's homed GLB is already in the Platform frame.
    assert bounds['jb_hand'][0][1] == pytest.approx(-.13608, abs=.001)
    # BB actuator frame rotates -90 deg around Z and translates to the rail.
    assert bounds['bb_pitch'][1][1] == pytest.approx(.4066, abs=.003)
    assert bounds['bb_hand'][0][1] == pytest.approx(.037, abs=.001)
    # Asset bounds, checked via the pre-2026-10-09 placement (-105.65 mm on X).
    # The runtime now adds BB_YAW_S_OFFSET_MM = +105.65 mm (hand on its true side).
    assert bounds['bb_hand'][0][0] - .10565 == pytest.approx(-.14192, abs=.001)
    assert bounds['bb_hand'][1][0] - .10565 == pytest.approx(-.04765, abs=.001)


def test_curved_surfaces_have_smooth_normals(asset):
    gltf, binary, _ = asset
    node = next(n for n in gltf['nodes'] if n['name'] == 'jb_outer')
    primitive = gltf['meshes'][node['mesh']]['primitives'][0]
    normals = values(gltf, binary, primitive['attributes']['NORMAL'])
    indices = [row[0] for row in values(gltf, binary, primitive['indices'])]
    smooth_faces = sum(
        max(abs(normals[indices[i]][j] - normals[indices[i + 1]][j]) for j in range(3)) > .001
        for i in range(0, len(indices), 3)
    )
    assert smooth_faces > len(indices) // 30, 'Curved CAD reverted to flat per-face shading'
