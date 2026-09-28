"""Build browser CAD from Onshape GLBs offline with sampled surface-error checks.

Blender 5.2 --background --python tools/build_gui_meshes.py --
    --source-dir temp/gui-robot-source
Raw Onshape GLBs are Z-up/metres. Source files are never modified. Flat surfaces
may have long edges: tolerance measures surface deviation, not triangle edge size.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion
from mathutils.bvhtree import BVHTree

ROOT = Path(__file__).resolve().parents[1]
# Registration from CAD export frames into each runtime rigid body's local frame.
SPECS = [
    ('jb_base', 'jugglebot/Base.glb', (0, 0, -.082)),
    ('jb_platform', 'jugglebot/Platform.glb', (0, 0, 0)),
    ('jb_outer', 'jugglebot/Leg Outer.glb', (0, 0, -.064)),
    ('jb_inner', 'jugglebot/Leg Inner.glb', (0, 0, -.70813)),
    ('jb_hand', 'jugglebot/Hand.glb', (0, 0, 0)),
    ('bb_base', 'ball-butler/Base.glb', (0, 0, -.069)),
    ('bb_yaw', 'ball-butler/Yaw Axis.glb', (0, 0, -.069)),
    ('bb_pitch', 'ball-butler/Pitch Axis.glb', (0, .041, -.0865)),
    ('bb_hand', 'ball-butler/Hand.glb', (.07015, 0, .211)),
]
# BB linear-actuator CAD frame has its origin at the rail centre and is
# rotated +90 degrees around Z relative to the pitch assembly. The runtime
# separately supplies the -105.65 mm lateral hand offset and pitch pivot.
BB_HAND_ROTATION = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])


def glb_source(path):
    data = path.read_bytes()
    magic, version, size = struct.unpack_from('<4sII', data)
    assert magic == b'glTF' and version == 2 and size == len(data)
    length = struct.unpack_from('<I', data, 12)[0]
    document = json.loads(data[20:20 + length])
    assert not document.get('extensionsRequired') and not document.get('images')
    assert all('uri' not in b for b in document['buffers'])
    return document, memoryview(data)[28 + length:], hashlib.sha256(data).hexdigest()


def accessor(doc, binary, index):
    a = doc['accessors'][index]
    v = doc['bufferViews'][a['bufferView']]
    dtype = np.dtype({5121: '<u1', 5123: '<u2', 5125: '<u4', 5126: '<f4'}[a['componentType']])
    width = {'SCALAR': 1, 'VEC3': 3, 'VEC4': 4}[a['type']]
    return np.ndarray((a['count'], width), dtype=dtype, buffer=binary,
                      offset=v.get('byteOffset', 0) + a.get('byteOffset', 0),
                      strides=(v.get('byteStride', dtype.itemsize * width), dtype.itemsize)).copy()


def occurrences(doc):
    def visit(index, parent):
        node = doc['nodes'][index]
        if 'matrix' in node:
            local = np.asarray(node['matrix']).reshape(4, 4).T
        else:
            q = node.get('rotation', [0, 0, 0, 1])
            local = np.array(Quaternion((q[3], *q[:3])).to_matrix().to_4x4())
            local[:3, :3] *= np.asarray(node.get('scale', [1, 1, 1]))
            local[:3, 3] = node.get('translation', [0, 0, 0])
        world = parent @ local
        if 'mesh' in node:
            yield node['mesh'], world, node.get('name', '')
        for child in node.get('children', []):
            yield from visit(child, world)
    for root in doc['scenes'][doc.get('scene', 0)]['nodes']:
        yield from visit(root, np.eye(4))


def mesh_arrays(mesh):
    mesh.calc_loop_triangles()
    points = np.empty(len(mesh.vertices) * 3, dtype=np.float32)
    mesh.vertices.foreach_get('co', points)
    faces = np.empty(len(mesh.loop_triangles) * 3, dtype=np.int32)
    mesh.loop_triangles.foreach_get('vertices', faces)
    return points.reshape(-1, 3), faces.reshape(-1, 3)


def material_groups(doc, binary, mesh_index):
    """Join CAD face primitives before collapse; omit exported construction lines."""
    groups = {}
    for primitive in doc['meshes'][mesh_index]['primitives']:
        if primitive.get('mode', 4) != 4:
            continue
        material = primitive.get('material')
        points, faces, count = groups.setdefault(material, ([], [], [0]))
        p = accessor(doc, binary, primitive['attributes']['POSITION'])
        indices = (accessor(doc, binary, primitive['indices']).ravel()
                   if 'indices' in primitive else np.arange(len(p)))
        points.append(p)
        faces.append(indices.reshape(-1, 3).astype(np.int32) + count[0])
        count[0] += len(p)
    return [(material, np.concatenate(p), np.concatenate(f))
            for material, (p, f, _) in groups.items()]


def samples(points, faces):
    # Deterministic coverage: vertices, face centres, edge midpoints and extrema.
    vi = np.linspace(0, len(points) - 1, min(len(points), 4096), dtype=int)
    fi = np.linspace(0, len(faces) - 1, min(len(faces), 4096), dtype=int)
    triangles = points[faces[fi]]
    return np.concatenate((points[vi], triangles.mean(axis=1),
                           (triangles[:, 0] + triangles[:, 1]) * .5,
                           points[np.argmin(points, axis=0)], points[np.argmax(points, axis=0)]))


def nearest_max(tree, points):
    return max(tree.find_nearest(p)[3] for p in points)


def simplify(points, faces, tolerance, cache):
    key = hashlib.sha256(points.tobytes() + faces.tobytes() + str(tolerance).encode() + b'v1').hexdigest()
    cached = cache / (key + '.npz')
    if cached.exists():
        with np.load(cached) as data:
            return data['points'], data['faces'], float(data['error'])
    if len(faces) <= 24:
        return points, faces, 0.
    mesh = bpy.data.meshes.new('component-source')
    mesh.from_pydata(points.tolist(), [], faces.tolist())
    # glTF splits vertices at shading seams. Weld before edge collapse.
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=1e-7)
    bm.to_mesh(mesh)
    bm.free()
    mesh.validate()
    clean_points, clean_faces = mesh_arrays(mesh)
    source_tree = BVHTree.FromPolygons(points.tolist(), faces.tolist(), all_triangles=True)
    source_samples = samples(points, faces)
    obj = bpy.data.objects.new('component-work', mesh)
    bpy.context.collection.objects.link(obj)
    bpy.context.view_layer.objects.active = obj
    ratio = min(1., max(24, min(8000, len(clean_faces) * .015)) / len(clean_faces))
    while True:
        modifier = obj.modifiers.new('Error bounded reduction', 'DECIMATE')
        modifier.ratio = ratio
        modifier.use_collapse_triangulate = True
        evaluated = obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
        reduced = evaluated.to_mesh()
        new_points, new_faces = mesh_arrays(reduced)
        if len(new_faces):
            tree = BVHTree.FromPolygons(new_points.tolist(), new_faces.tolist(), all_triangles=True)
            error = max(nearest_max(tree, source_samples), nearest_max(source_tree, samples(new_points, new_faces)))
        else:
            error = math.inf
        evaluated.to_mesh_clear()
        obj.modifiers.clear()
        if error <= tolerance:
            break
        if ratio == 1:
            # Do not compromise source accuracy if welding touched an unusual mesh.
            new_points, new_faces, error = points, faces, 0.
            break
        ratio = min(1., ratio * 2)
    bpy.data.objects.remove(obj, do_unlink=True)
    bpy.data.meshes.remove(mesh)
    np.savez_compressed(cached, points=new_points, faces=new_faces, error=error)
    return new_points, new_faces, error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, default=ROOT / 'temp/gui-robot-source')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'ros_ws/gui/assets/robots')
    parser.add_argument('--tolerance-mm', type=float, default=.75)
    args = parser.parse_args(sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else [])
    assert 0 < args.tolerance_mm <= 1
    cache = ROOT / 'temp/gui-mesh-cache'
    cache.mkdir(parents=True, exist_ok=True)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    material = bpy.data.materials.new('CAD vertex colours')
    material.use_nodes = True
    shader = material.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Metallic'].default_value = .25
    shader.inputs['Roughness'].default_value = .55
    color_node = material.node_tree.nodes.new('ShaderNodeVertexColor')
    color_node.layer_name = 'Color'
    material.node_tree.links.new(color_node.outputs['Color'], shader.inputs['Base Color'])
    manifest = {'generator': 'tools/build_gui_meshes.py', 'tolerance_mm': args.tolerance_mm,
                'validation': 'Bidirectional sampled surface distance; vertices, centroids, edge midpoints and extrema. Not a formal Hausdorff bound.', 'parts': {}}
    for name, filename, offset in SPECS:
        print('BUILD', name, filename, flush=True)
        doc, binary, source_hash = glb_source(args.source_dir / filename)
        processed = {}
        verts, triangles, colors = [], [], []
        count = 0
        max_error = 0.
        source_triangles = 0
        for mesh_index, matrix, label in occurrences(doc):
            scale = np.linalg.svd(matrix[:3, :3], compute_uv=False).max()
            assert abs(scale - 1) < 1e-4, 'Expected CAD in metres with rigid instance transforms'
            if mesh_index not in processed:
                processed[mesh_index] = []
                for material_index, points, faces in material_groups(doc, binary, mesh_index):
                    p, f, error = simplify(points, faces, args.tolerance_mm * .001 / scale, cache)
                    processed[mesh_index].append((p, f, error, len(faces), material_index))
                if len(processed) % 50 == 0:
                    print(name, 'components', len(processed), flush=True)
            for points, faces, error, original_count, material_index in processed[mesh_index]:
                transformed = points @ matrix[:3, :3].T + matrix[:3, 3]
                if name == 'bb_hand':
                    transformed = transformed @ BB_HAND_ROTATION.T
                transformed += offset
                verts.append(transformed)
                triangles.append(faces + count)
                count += len(points)
                source_triangles += original_count
                max_error = max(max_error, error * scale)
                src_mat = doc['materials'][material_index] if material_index is not None else {}
                rgba = src_mat.get('pbrMetallicRoughness', {}).get('baseColorFactor', [1, 1, 1, 1])
                colors.append(np.tile([*rgba[:3], 1.], (len(faces) * 3, 1)))
        mesh = bpy.data.meshes.new(name)
        mesh.from_pydata(np.concatenate(verts).tolist(), [], np.concatenate(triangles).tolist())
        # All instances are baked into one rigid mesh, one material / draw call.
        attribute = mesh.color_attributes.new(name='Color', type='FLOAT_COLOR', domain='CORNER')
        attribute.data.foreach_set('color', np.concatenate(colors).astype(np.float32).ravel())
        # Some CAD faces contain duplicate or degenerate indices. Validation also
        # carries corner colours along with any removed faces.
        mesh.validate(clean_customdata=False)
        mesh.set_sharp_from_angle(angle=math.radians(30))
        for poly in mesh.polygons:
            poly.use_smooth = True
        mesh.materials.append(material)
        obj = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(obj)
        mesh.calc_loop_triangles()
        total = len(mesh.loop_triangles)
        manifest['parts'][name] = dict(source=filename, sha256=source_hash, source_triangles=source_triangles,
            triangles=total, sampled_max_error_mm=round(max_error * 1000, 6),
            registration_offset_m=list(offset), unique_components=len(processed))
        if name == 'bb_hand':
            manifest['parts'][name]['registration_rotation_z_degrees'] = -90
        print('DONE', name, source_triangles, '->', total, 'error mm', max_error * 1000, flush=True)
    output = args.output_dir / 'robot-parts.glb'
    bpy.ops.export_scene.gltf(filepath=str(output), export_format='GLB', export_yup=True,
                              export_animations=False, export_cameras=False, export_lights=False)
    manifest['bytes'] = output.stat().st_size
    manifest['scene_triangles'] = sum(p['triangles'] * (6 if n in ('jb_outer', 'jb_inner') else 1) for n, p in manifest['parts'].items())
    (args.output_dir / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')


if __name__ == '__main__':
    main()
