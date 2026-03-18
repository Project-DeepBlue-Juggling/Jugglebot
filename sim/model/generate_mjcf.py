"""Generate MuJoCo MJCF XML for Jugglebot Stewart platform.

Reads geometry from hardware_config.yaml and produces jugglebot.xml.

Usage:
    python sim/model/generate_mjcf.py          # from repo root
    python model/generate_mjcf.py              # from sim/
"""

import numpy as np
import yaml
import os
import xml.etree.ElementTree as ET
from xml.dom import minidom


def load_hardware_config():
    """Load hardware_config.yaml, trying both repo-root and sim/ working dirs."""
    candidates = [
        os.path.join(os.path.dirname(__file__), '..', '..', 'config', 'hardware_config.yaml'),
        os.path.join('config', 'hardware_config.yaml'),
        os.path.join('..', 'config', 'hardware_config.yaml'),
    ]
    for path in candidates:
        if os.path.exists(path):
            with open(path) as f:
                return yaml.safe_load(f)
    raise FileNotFoundError("Cannot find config/hardware_config.yaml")


def rotation_from_z_to_dir(direction):
    """Compute quaternion (w,x,y,z) that rotates [0,0,1] to `direction`."""
    d = np.array(direction, dtype=float)
    d /= np.linalg.norm(d)
    z = np.array([0.0, 0.0, 1.0])
    dot = np.dot(z, d)

    if dot > 0.99999:
        return np.array([1.0, 0.0, 0.0, 0.0])
    if dot < -0.99999:
        return np.array([0.0, 1.0, 0.0, 0.0])

    axis = np.cross(z, d)
    axis /= np.linalg.norm(axis)
    angle = np.arccos(np.clip(dot, -1, 1))
    half = angle / 2
    return np.array([np.cos(half), *(np.sin(half) * axis)])


def fmt(arr, precision=6):
    """Format an array as a space-separated string."""
    if isinstance(arr, (list, tuple, np.ndarray)):
        return ' '.join(f'{v:.{precision}f}' for v in arr)
    return f'{arr:.{precision}f}'


def generate_mjcf(config, mesh_dir=None):
    """Generate MJCF XML string from hardware config dict.

    Args:
        config: Hardware config dict from YAML.
        mesh_dir: Path to directory containing STL meshes. If None, auto-detected
                  relative to this script. Meshes are included only if present.
    """
    geom = config['jugglebot_geometry']
    dyn = config['dynamics']

    # ---- Detect available meshes ----
    if mesh_dir is None:
        mesh_dir = os.path.join(os.path.dirname(__file__), 'meshes')
    mesh_files = {
        'base': os.path.join(mesh_dir, 'base.stl'),
        'platform': os.path.join(mesh_dir, 'platform.stl'),
        'leg_outer': os.path.join(mesh_dir, 'leg_outer.stl'),
        'leg_inner': os.path.join(mesh_dir, 'leg_inner.stl'),
        'hand': os.path.join(mesh_dir, 'hand.stl'),
    }
    has_meshes = {k: os.path.exists(v) for k, v in mesh_files.items()}

    # ---- Extract geometry values (convert mm → m) ----
    initial_height_m = geom['initial_height_mm'] / 1000.0
    leg_stroke_m = geom['leg_stroke_mm'] / 1000.0
    ball_joint_offset_m = geom['ball_joint_offset_mm'] / 1000.0
    hand_stroke_m = geom['hand_stroke_mm'] / 1000.0   # 0.355 m
    hand_radius_m = geom['hand_radius_mm'] / 1000.0   # ~0.035 m

    # Hand dynamics
    teensy_traj = config['teensy_trajectory']
    hand_mass_kg = teensy_traj['inertia_hand_only_kg']  # 0.281 kg
    hand_bottom_z_m = -0.135  # Bottom of travel in platform-local Z (mechanical constraint)

    base_nodes_m = np.array(geom['base_nodes_mm']) / 1000.0  # (6, 3)
    plat_nodes_m = np.array(geom['init_plat_nodes_mm']) / 1000.0  # (6, 3) in platform frame
    init_leg_lengths_m = np.array(geom['init_leg_lengths_mm']) / 1000.0

    # Platform nodes in world frame at home position
    plat_nodes_world_m = plat_nodes_m + np.array([0, 0, initial_height_m])

    # ---- Compute leg geometry ----
    leg_vecs = plat_nodes_world_m - base_nodes_m  # (6, 3)
    leg_lengths = np.linalg.norm(leg_vecs, axis=1)  # (6,)
    leg_dirs = leg_vecs / leg_lengths[:, None]  # (6, 3) unit vectors

    # Geometric home leg length (from node positions)
    # This is used as the slide joint home value
    # The init_leg_lengths from hardware are close but not identical due to
    # measurement + ball joint offset differences
    home_slide = leg_lengths  # (6,) geometric home lengths

    # ---- Platform dynamics (convert to SI) ----
    platform_mass = dyn['platform_mass_kg']
    com_offset_m = np.array(dyn['platform_com_offset_mm']) / 1000.0
    inertia = dyn['platform_inertia_tensor_kgmm2']
    # Convert kg·mm² → kg·m² (÷ 1e6)
    Ixx = inertia['Ixx'] * 1e-6
    Iyy = inertia['Iyy'] * 1e-6
    Izz = inertia['Izz'] * 1e-6
    Ixy = inertia['Ixy'] * 1e-6
    Ixz = inertia['Ixz'] * 1e-6
    Iyz = inertia['Iyz'] * 1e-6

    gravity = config['physics']['gravity_mps2']

    # ---- Build XML ----
    mujoco = ET.Element('mujoco', model='jugglebot')

    # Compiler
    ET.SubElement(mujoco, 'compiler', angle='radian', coordinate='local')

    # Simulation options
    ET.SubElement(mujoco, 'option', **{
        'timestep': '0.001',      # 1000 Hz (needed for stiff actuators with realistic masses)
        'gravity': f'0 0 {-gravity}',
        'solver': 'Newton',
        'iterations': '100',
        'tolerance': '1e-10',
        'impratio': '1',
    })

    # Size
    ET.SubElement(mujoco, 'size', nconmax='100', njmax='500')

    # Visual
    visual = ET.SubElement(mujoco, 'visual')
    ET.SubElement(visual, 'global', azimuth='135', elevation='-30')
    ET.SubElement(visual, 'quality', shadowsize='2048')

    # Assets
    asset = ET.SubElement(mujoco, 'asset')
    ET.SubElement(asset, 'texture', name='grid', type='2d', builtin='checker',
                  width='512', height='512', rgb1='0.9 0.9 0.9', rgb2='0.7 0.7 0.7')
    ET.SubElement(asset, 'material', name='grid_mat', texture='grid',
                  texrepeat='8 8', reflectance='0.1')
    ET.SubElement(asset, 'material', name='platform_mat', rgba='0.3 0.3 0.8 0.9')
    ET.SubElement(asset, 'material', name='base_mat', rgba='0.4 0.4 0.4 0.8')
    ET.SubElement(asset, 'material', name='leg_mat', rgba='0.7 0.7 0.7 1.0')
    ET.SubElement(asset, 'material', name='joint_mat', rgba='0.9 0.2 0.2 1.0')
    ET.SubElement(asset, 'material', name='hand_mat', rgba='0.8 0.6 0.2 0.9')
    ET.SubElement(asset, 'material', name='ball_mat', rgba='1.0 0.5 0.0 1.0')
    # STL meshes (included only when files are present in meshes/)
    # All STLs exported from Onshape in mm — scale to meters
    mesh_scale = '0.001 0.001 0.001'
    if has_meshes['base']:
        ET.SubElement(asset, 'mesh', name='base_mesh', file='meshes/base.stl',
                      scale=mesh_scale)
    if has_meshes['platform']:
        ET.SubElement(asset, 'mesh', name='platform_mesh', file='meshes/platform.stl',
                      scale=mesh_scale)
    if has_meshes['leg_outer']:
        ET.SubElement(asset, 'mesh', name='leg_outer_mesh', file='meshes/leg_outer.stl',
                      scale=mesh_scale)
    if has_meshes['leg_inner']:
        ET.SubElement(asset, 'mesh', name='leg_inner_mesh', file='meshes/leg_inner.stl',
                      scale=mesh_scale)
    if has_meshes['hand']:
        ET.SubElement(asset, 'mesh', name='hand_mesh', file='meshes/hand.stl',
                      scale=mesh_scale)

    # Default classes
    default = ET.SubElement(mujoco, 'default')
    ET.SubElement(default, 'joint', damping='50', armature='0.01')
    leg_default = ET.SubElement(default, 'default', **{'class': 'leg'})
    ET.SubElement(leg_default, 'joint', damping='200')
    ET.SubElement(leg_default, 'geom', type='capsule', size='0.012', material='leg_mat',
                  contype='0', conaffinity='0')  # No collisions

    # ---- Worldbody ----
    worldbody = ET.SubElement(mujoco, 'worldbody')

    # Ground plane
    ET.SubElement(worldbody, 'geom', name='floor', type='plane',
                  size='1.0 1.0 0.01', material='grid_mat', pos='0 0 -0.082')
    ET.SubElement(worldbody, 'light', directional='true', diffuse='0.8 0.8 0.8',
                  pos='0 0 2', dir='0 0 -1')

    # Base frame visual
    if has_meshes['base']:
        # STL origin is 82mm below the assembly centre (ball joint plane)
        ET.SubElement(worldbody, 'geom', name='base_visual', type='mesh',
                      mesh='base_mesh', pos='0 0 -0.082',
                      material='base_mat', contype='0', conaffinity='0')
    else:
        base_ring_r = float(geom['base_radius_mm']) / 1000.0
        ET.SubElement(worldbody, 'geom', name='base_ring', type='cylinder',
                      size=f'{base_ring_r:.4f} 0.005', pos='0 0 -0.005',
                      material='base_mat', contype='0', conaffinity='0')
    for i, bn in enumerate(base_nodes_m):
        ET.SubElement(worldbody, 'site', name=f'base_site_{i}',
                      pos=fmt(bn), size='0.015', rgba='0.9 0.2 0.2 1')

    # ---- Platform body (free joint, 6 DoF) ----
    platform = ET.SubElement(worldbody, 'body', name='platform',
                             pos=f'0 0 {initial_height_m:.6f}')
    ET.SubElement(platform, 'freejoint', name='platform_joint')
    ET.SubElement(platform, 'inertial',
                  pos=fmt(com_offset_m),
                  mass=str(platform_mass),
                  fullinertia=f'{Ixx:.9f} {Iyy:.9f} {Izz:.9f} {Ixy:.9f} {Ixz:.9f} {Iyz:.9f}')
    # Platform visual
    if has_meshes['platform']:
        # STL origin is at the platform body frame origin
        ET.SubElement(platform, 'geom', name='platform_visual', type='mesh',
                      mesh='platform_mesh', pos='0 0 0',
                      material='platform_mat', contype='0', conaffinity='0')
    else:
        plat_radius_m = float(geom['plat_radius_mm']) / 1000.0
        ET.SubElement(platform, 'geom', name='platform_top', type='cylinder',
                      size=f'{plat_radius_m:.4f} 0.008', pos='0 0 0',
                      material='platform_mat', contype='0', conaffinity='0')
    # Site at platform origin (body frame) — used for sensors that need the
    # body frame position, not the inertial/CoM frame that MuJoCo body sensors return
    ET.SubElement(platform, 'site', name='platform_origin',
                  pos='0 0 0', size='0.005', rgba='1 1 1 0')
    # Sites at platform nodes (for visualization and constraint reference)
    for i, pn in enumerate(plat_nodes_m):
        ET.SubElement(platform, 'site', name=f'plat_site_{i}',
                      pos=fmt(pn), size='0.012', rgba='0.2 0.8 0.2 1')

    # ---- Hand body (child of platform) ----
    # Hand is a 1-DOF prismatic actuator on the platform's local Z axis.
    # Bottom of travel at Z = -135 mm in platform frame.
    # Stroke: 355 mm, so top of travel at Z = +220 mm.
    hand_body = ET.SubElement(platform, 'body', name='hand',
                               pos=f'0 0 {hand_bottom_z_m:.6f}')
    ET.SubElement(hand_body, 'inertial',
                  pos='0 0 0.055', mass=str(hand_mass_kg),
                  diaginertia='0.000200 0.000200 0.000050')
    # Prismatic joint along platform-local Z
    ET.SubElement(hand_body, 'joint', name='hand_slide',
                  type='slide', axis='0 0 1', limited='true',
                  range=f'0 {hand_stroke_m:.6f}',
                  damping='50', armature='0.001')
    # Hand visual
    if has_meshes['hand']:
        # Mesh rotation: -90° X, +90° Y, +90° Y = quat (w,x,y,z) = (0, 0, 0.707107, 0.707107)
        # Mesh origin is at platform centroid; offset to hand body frame
        ET.SubElement(hand_body, 'geom', name='hand_visual',
                      type='mesh', mesh='hand_mesh',
                      pos=f'0 -0.065 0',
                      quat='0 0 0.707107 0.707107',
                      material='hand_mat', contype='0', conaffinity='0',
                      mass='0')  # mass already set via inertial
    else:
        # Fallback: stacked cylinders approximating truncated cone
        ET.SubElement(hand_body, 'geom', name='hand_cone_base',
                      type='cylinder', size=f'{hand_radius_m:.4f} 0.030',
                      pos='0 0 0.080', material='hand_mat',
                      contype='0', conaffinity='0', mass='0')
        ET.SubElement(hand_body, 'geom', name='hand_cone_tip',
                      type='cylinder', size=f'{hand_radius_m * 0.5:.4f} 0.025',
                      pos='0 0 0.030', material='hand_mat',
                      contype='0', conaffinity='0', mass='0')
    # Collision disc at the bottom of the cone — catches the ball
    # The ball lands on this disc and stops (dead contact).
    # Separate from the visual mesh so collision shape stays simple.
    ET.SubElement(hand_body, 'geom', name='hand_catch_disc',
                  type='cylinder', size=f'{hand_radius_m:.4f} 0.003',
                  pos='0 0 0.003',
                  contype='2', conaffinity='2',
                  solref='0.005 2.0',       # overdamped (no bounce)
                  solimp='0.99 0.99 0.001',
                  friction='1.0 0.005 0.0001',
                  rgba='0 0 0 0', mass='0')  # invisible, massless
    # Site at the cone opening centre — used for capture proximity check
    # The cone is ~40 mm tall, opening at the top
    ET.SubElement(hand_body, 'site', name='hand_opening',
                  pos='0 0 0.040', size='0.010', rgba='1.0 0.8 0.0 1')

    # ---- Leg bodies (nested structure) ----
    # Outer body: at base node with ball joint (2 hinges)
    # Inner body: offset by geometric home leg length along local Z,
    #             with slide joint for extension beyond home.
    #
    # This ensures that at compile time (all joints at 0), the inner body
    # (leg tip) is already at the platform attachment point. MuJoCo computes
    # equality constraint anchors from this reference configuration, so the
    # connect constraint anchor on the leg body is correctly at [0,0,0].
    #
    # Slide range: [-0.005, stroke + 0.005] to accommodate the ~1.5mm
    # discrepancy between geometric and measured home leg lengths.
    slide_margin_m = 0.005
    for i in range(6):
        quat = rotation_from_z_to_dir(leg_dirs[i])
        home_len = home_slide[i]

        # Outer body at base node with ball joint
        leg_base = ET.SubElement(worldbody, 'body', name=f'leg_{i}_base',
                                 pos=fmt(base_nodes_m[i]),
                                 quat=fmt(quat))

        # Outer leg: tube (~0.5 kg) + motor (~0.89 kg) = 1.39 kg
        # CoM at ~35% up the leg from the base node
        outer_mass = 1.39
        outer_com_z = home_len * 0.35
        # Approximate as thin rod (length ~0.65m, radius ~0.02m)
        outer_len = home_len
        outer_Izz = outer_mass * 0.02**2 / 2               # axial
        outer_Ixx = outer_mass * (3 * 0.02**2 + outer_len**2) / 12  # transverse
        ET.SubElement(leg_base, 'inertial',
                      pos=f'0 0 {outer_com_z:.6f}', mass=f'{outer_mass}',
                      diaginertia=f'{outer_Ixx:.6f} {outer_Ixx:.6f} {outer_Izz:.6f}')

        # Ball joint at base (2 hinges — no axial spin needed)
        ET.SubElement(leg_base, 'joint', name=f'leg_{i}_hinge_x',
                      type='hinge', axis='1 0 0', limited='false',
                      **{'class': 'leg'})
        ET.SubElement(leg_base, 'joint', name=f'leg_{i}_hinge_y',
                      type='hinge', axis='0 1 0', limited='false',
                      **{'class': 'leg'})

        # Leg outer visual
        if has_meshes['leg_outer']:
            # STL origin is 64mm below ball joint centre, Z along leg axis
            ET.SubElement(leg_base, 'geom', name=f'leg_{i}_outer_visual',
                          type='mesh', mesh='leg_outer_mesh', pos='0 0 -0.064',
                          material='leg_mat', contype='0', conaffinity='0')
        else:
            ET.SubElement(leg_base, 'geom', name=f'leg_{i}_geom',
                          type='capsule', fromto=f'0 0 0 0 0 {home_len:.6f}',
                          size='0.012', **{'class': 'leg'})

        # Inner body: offset to platform attachment point
        # At compile time (slide=0), this body is at:
        #   base_node + R_leg * [0, 0, home_len] = plat_node_world
        leg_tip = ET.SubElement(leg_base, 'body', name=f'leg_{i}',
                                pos=f'0 0 {home_len:.6f}')
        # Inner leg (sliding tube): ~0.1 kg
        # CoM near the body origin (platform attachment end)
        inner_mass = 0.1
        inner_Izz = inner_mass * 0.015**2 / 2
        inner_Ixx = inner_mass * (3 * 0.015**2 + 0.3**2) / 12  # ~300mm visible length
        ET.SubElement(leg_tip, 'inertial',
                      pos='0 0 0', mass=f'{inner_mass}',
                      diaginertia=f'{inner_Ixx:.6f} {inner_Ixx:.6f} {inner_Izz:.6f}')

        # Prismatic joint: extension beyond home length
        # slide=0 → leg at geometric home length (home position)
        # slide>0 → leg extended, platform pushed further
        ET.SubElement(leg_tip, 'joint', name=f'leg_{i}_slide',
                      type='slide', axis='0 0 1', limited='true',
                      range=f'{-slide_margin_m:.6f} {leg_stroke_m + slide_margin_m:.6f}',
                      **{'class': 'leg'})

        # Leg inner visual
        if has_meshes['leg_inner']:
            # STL origin is 704.47mm below the joint centre along leg Z.
            # Inner body frame origin is at platform attachment point.
            ET.SubElement(leg_tip, 'geom', name=f'leg_{i}_inner_visual',
                          type='mesh', mesh='leg_inner_mesh',
                          pos='0 0 -0.70447',
                          material='leg_mat', contype='0', conaffinity='0')

        # Site at leg tip (for visualization)
        ET.SubElement(leg_tip, 'site', name=f'leg_{i}_tip',
                      pos='0 0 0', size='0.010',
                      rgba='0.9 0.9 0.2 1')

    # ---- Ball body (free-flying, for catch simulation) ----
    ball_body = ET.SubElement(worldbody, 'body', name='ball', pos='0 0 5')
    ET.SubElement(ball_body, 'freejoint', name='ball_joint')
    # contype=3, conaffinity=3: collides with ground (bit 0) AND hand (bit 1)
    # solref overdamped: ball stops dead on contact (no bounce)
    ET.SubElement(ball_body, 'geom', name='ball_geom', type='sphere',
                  size='0.02', mass='0.043', material='ball_mat',
                  contype='3', conaffinity='3',
                  solref='0.005 2.0',
                  solimp='0.99 0.99 0.001',
                  friction='1.0 0.005 0.0001')
    ET.SubElement(ball_body, 'site', name='ball_site', pos='0 0 0',
                  size='0.005', rgba='1 1 1 0')

    # ---- Equality constraints ----
    equality = ET.SubElement(mujoco, 'equality')
    for i in range(6):
        # Connect: anchor on platform (platform node in platform frame) = leg_i body origin
        # body1=leg_i, body2=platform: anchor is in leg_i frame, constrains to platform origin
        # Actually: we want leg tip = platform node
        # Use body1=platform, body2=leg_i, anchor=platform_node_in_platform_frame
        # This constrains: platform.transform(anchor) = leg_i.origin
        ET.SubElement(equality, 'connect',
                      body1='platform', body2=f'leg_{i}',
                      anchor=fmt(plat_nodes_m[i]),
                      solref='0.005 1',   # tight constraint
                      solimp='0.99 0.99 0.001')

    # Ball-to-hand weld constraint (disabled by default, activated on capture)
    # relpose places ball at the hand opening site when active
    ET.SubElement(equality, 'weld', name='ball_catch',
                  body1='hand', body2='ball',
                  relpose='0 0 0.040 1 0 0 0',   # at hand_opening site
                  active='false',
                  solref='0.005 1',
                  solimp='0.99 0.99 0.001')

    # ---- Actuators ----
    actuator = ET.SubElement(mujoco, 'actuator')
    for i in range(6):
        # Position actuator on the prismatic (slide) joint
        # ctrl=0 → home position; ctrl=0.280 → max extension
        ET.SubElement(actuator, 'position', name=f'act_{i}',
                      joint=f'leg_{i}_slide',
                      kp='200000',     # stiff position tracking (high-inertia motors)
                      kv='600',        # near-critical damping for ~0.3kg effective mass
                      ctrlrange=f'{-slide_margin_m:.6f} {leg_stroke_m + slide_margin_m:.6f}',
                      ctrllimited='true')
    # Hand actuator: position control matching ODrive response
    # kp=10000 + kv=200 produces settling similar to real ODrive at pos_gain=35
    ET.SubElement(actuator, 'position', name='act_hand',
                  joint='hand_slide',
                  kp='10000', kv='200',
                  ctrlrange=f'0 {hand_stroke_m:.6f}',
                  ctrllimited='true')

    # ---- Keyframe: home position ----
    keyframe = ET.SubElement(mujoco, 'keyframe')
    # qpos order: platform free joint (7: pos xyz + quat wxyz),
    #             hand_slide (1),
    #             then for each leg: hinge_x, hinge_y, slide (3 per leg),
    #             ball free joint (7: pos xyz + quat wxyz)
    # At home: platform at [0,0,height] with identity quat, hand at 0, all hinges=0, slides=0,
    #          ball parked at (0,0,5) above scene
    platform_qpos = [0, 0, initial_height_m, 1, 0, 0, 0]  # pos + quat (wxyz)
    hand_qpos = [0]  # hand at bottom of travel
    leg_qpos = []
    for i in range(6):
        leg_qpos.extend([0, 0, 0])  # hinge_x, hinge_y, slide (0 = home)
    ball_qpos = [0, 0, 5, 1, 0, 0, 0]  # parked high above scene

    all_qpos = platform_qpos + hand_qpos + leg_qpos + ball_qpos
    ET.SubElement(keyframe, 'key', name='home',
                  qpos=' '.join(f'{v:.6f}' for v in all_qpos))

    # ---- Sensors (for reading state) ----
    sensor = ET.SubElement(mujoco, 'sensor')
    for i in range(6):
        ET.SubElement(sensor, 'jointpos', name=f'slide_pos_{i}', joint=f'leg_{i}_slide')
        ET.SubElement(sensor, 'jointvel', name=f'slide_vel_{i}', joint=f'leg_{i}_slide')
    # Platform pose via site-based sensors (body frame origin, NOT CoM frame)
    # MuJoCo body-based framepos/framequat return the inertial (CoM) frame,
    # so we use a site at the body origin instead.
    ET.SubElement(sensor, 'framepos', name='platform_pos', objtype='site', objname='platform_origin')
    ET.SubElement(sensor, 'framequat', name='platform_quat', objtype='site', objname='platform_origin')
    # Velocities still use the body (CoM frame is fine for angular velocity)
    ET.SubElement(sensor, 'framelinvel', name='platform_linvel', objtype='site', objname='platform_origin')
    ET.SubElement(sensor, 'frameangvel', name='platform_angvel', objtype='site', objname='platform_origin')
    # Hand sensors
    ET.SubElement(sensor, 'jointpos', name='hand_slide_pos', joint='hand_slide')
    ET.SubElement(sensor, 'jointvel', name='hand_slide_vel', joint='hand_slide')
    # Ball sensors
    ET.SubElement(sensor, 'framepos', name='ball_pos', objtype='site', objname='ball_site')
    ET.SubElement(sensor, 'framelinvel', name='ball_vel', objtype='site', objname='ball_site')

    return mujoco


def prettify(element):
    """Return a pretty-printed XML string."""
    rough = ET.tostring(element, encoding='unicode')
    parsed = minidom.parseString(rough)
    lines = parsed.toprettyxml(indent='  ').split('\n')
    # Remove blank lines and xml declaration
    return '\n'.join(line for line in lines
                     if line.strip() and not line.startswith('<?xml'))


def main():
    config = load_hardware_config()
    root = generate_mjcf(config)
    xml_str = prettify(root)

    # Add XML declaration at the top
    xml_str = '<?xml version="1.0" encoding="utf-8"?>\n' + xml_str

    out_path = os.path.join(os.path.dirname(__file__), 'jugglebot.xml')
    with open(out_path, 'w') as f:
        f.write(xml_str)
    print(f"Generated {out_path}")

    # Print summary
    geom = config['jugglebot_geometry']
    base_nodes = np.array(geom['base_nodes_mm'])
    plat_nodes = np.array(geom['init_plat_nodes_mm'])
    height = geom['initial_height_mm']
    plat_world = plat_nodes + [0, 0, height]
    leg_vecs = plat_world - base_nodes
    leg_lens = np.linalg.norm(leg_vecs, axis=1)

    print(f"\nGeometric home leg lengths (mm): {leg_lens}")
    print(f"Config init_leg_lengths + offset (mm): "
          f"{np.array(geom['init_leg_lengths_mm']) + geom['ball_joint_offset_mm']}")
    print(f"Platform height: {height} mm")
    print(f"Leg stroke: {geom['leg_stroke_mm']} mm")
    print(f"Slide range per leg: [{leg_lens[0]:.3f}, {leg_lens[0] + geom['leg_stroke_mm']:.3f}] mm")


if __name__ == '__main__':
    main()
