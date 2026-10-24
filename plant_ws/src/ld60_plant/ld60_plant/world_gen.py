"""Builds the plant world from worlds/plant.sdf + config/plant.yaml.

The template contains a `<!-- @@CELLS@@ -->` marker; this module inserts the 4 production cells
(machine, conveyor, arm pedestal) and the recycled box pool there, so the layout lives in one place
(plant.yaml). Pure functions, no ROS imports.
"""
import math

STORAGE_X0 = -30.0      # box pool parking area, far outside the hall


def conveyor_geometry(cell, prod):
    """Pose, length and pick/spawn points of a cell conveyor."""
    (sx, sy), (ex, ey) = cell['conveyor']['start'], cell['conveyor']['end']
    L = math.hypot(ex - sx, ey - sy)
    ux, uy = (ex - sx) / L, (ey - sy) / L
    yaw = math.atan2(uy, ux)
    half_box = prod['box']['size'][1] / 2          # boxes travel along their y size
    pick = (ex - ux * (0.002 + half_box), ey - uy * (0.002 + half_box))   # box resting against the end stop
    spawn = (sx + ux * 0.12, sy + uy * 0.12)
    return {'cx': (sx + ex) / 2, 'cy': (sy + ey) / 2, 'yaw': yaw, 'length': L, 'u': (ux, uy),
            'pick': pick, 'spawn': spawn}


def box_name(lane, k):
    return f'box_{lane}_{k:02d}'


def storage_pose(lane_index, k):
    return (STORAGE_X0 - 0.25 * k, -3.0 + 1.0 * lane_index, 0.05)


def _hex_rgb(h):
    h = h.lstrip('#')
    return tuple(int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def conveyor_sdf(name, g, top, color):
    L, w = g['length'], 0.26
    r, gg, b = _hex_rgb(color)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{g['cx']:.4f} {g['cy']:.4f} -0.002 0 0 {g['yaw']:.6f}</pose>
      <link name="belt_link">
        <collision name="belt">
          <pose>0 0 {top - 0.015:.4f} 0 0 0</pose>
          <geometry><box><size>{L:.3f} {w} 0.03</size></box></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>150</mu2><fdir1>0 1 0</fdir1></ode></friction></surface>
        </collision>
        <collision name="guide_l"><pose>0 {w / 2 + 0.01} {top + 0.02:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} 0.02 0.04</size></box></geometry></collision>
        <collision name="guide_r"><pose>0 {-w / 2 - 0.01} {top + 0.02:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} 0.02 0.04</size></box></geometry></collision>
        <collision name="end_stop"><pose>{L / 2 + 0.006:.4f} 0 {top + 0.03:.4f} 0 0 0</pose><geometry><box><size>0.012 {w + 0.04} 0.06</size></box></geometry></collision>
        <collision name="frame"><pose>0 0 {(top - 0.03) / 2:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} {w - 0.04} {top - 0.03:.3f}</size></box></geometry></collision>
        <visual name="frame"><pose>0 0 {(top - 0.03) / 2:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} {w - 0.04} {top - 0.03:.3f}</size></box></geometry>
          <material><ambient>0.20 0.22 0.25 1</ambient><diffuse>0.20 0.22 0.25 1</diffuse></material></visual>
        <visual name="belt"><pose>0 0 {top - 0.015:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} {w} 0.03</size></box></geometry>
          <material><ambient>0.06 0.06 0.07 1</ambient><diffuse>0.06 0.06 0.07 1</diffuse></material></visual>
        <visual name="guide_l"><pose>0 {w / 2 + 0.01} {top + 0.02:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} 0.02 0.04</size></box></geometry>
          <material><ambient>{r:.2f} {gg:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {gg:.2f} {b:.2f} 1</diffuse></material></visual>
        <visual name="guide_r"><pose>0 {-w / 2 - 0.01} {top + 0.02:.4f} 0 0 0</pose><geometry><box><size>{L:.3f} 0.02 0.04</size></box></geometry>
          <material><ambient>{r:.2f} {gg:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {gg:.2f} {b:.2f} 1</diffuse></material></visual>
        <visual name="end_stop"><pose>{L / 2 + 0.006:.4f} 0 {top + 0.03:.4f} 0 0 0</pose><geometry><box><size>0.012 {w + 0.04} 0.06</size></box></geometry>
          <material><ambient>0.85 0.15 0.15 1</ambient><diffuse>0.85 0.15 0.15 1</diffuse></material></visual>
      </link>
      <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
        <link>belt_link</link>
        <odometry_publish_frequency>1</odometry_publish_frequency>
      </plugin>
    </model>"""


def machine_sdf(name, g, color):
    ux, uy = g['u']
    sx = g['cx'] - ux * (g['length'] / 2 + 0.45)
    sy = g['cy'] - uy * (g['length'] / 2 + 0.45)
    r, gg, b = _hex_rgb(color)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{sx:.4f} {sy:.4f} 0 0 0 {g['yaw']:.6f}</pose>
      <link name="link">
        <collision name="body"><pose>0 0 0.75 0 0 0</pose><geometry><box><size>0.8 0.9 1.5</size></box></geometry></collision>
        <visual name="body"><pose>0 0 0.75 0 0 0</pose><geometry><box><size>0.8 0.9 1.5</size></box></geometry>
          <material><ambient>0.82 0.84 0.86 1</ambient><diffuse>0.82 0.84 0.86 1</diffuse></material></visual>
        <visual name="band"><pose>0 0 1.35 0 0 0</pose><geometry><box><size>0.81 0.91 0.12</size></box></geometry>
          <material><ambient>{r:.2f} {gg:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {gg:.2f} {b:.2f} 1</diffuse></material></visual>
        <visual name="outlet"><pose>0.401 0 0.86 0 0 0</pose><geometry><box><size>0.004 0.32 0.2</size></box></geometry>
          <material><ambient>0.05 0.05 0.06 1</ambient><diffuse>0.05 0.05 0.06 1</diffuse></material></visual>
        <visual name="screen"><pose>0.25 0.451 1.05 0 0 0</pose><geometry><box><size>0.22 0.004 0.15</size></box></geometry>
          <material><ambient>0.1 0.5 0.9 1</ambient><diffuse>0.1 0.5 0.9 1</diffuse><emissive>0.05 0.3 0.6 1</emissive></material></visual>
        <visual name="lamp"><pose>-0.3 -0.3 1.62 0 0 0</pose><geometry><cylinder><radius>0.04</radius><length>0.24</length></cylinder></geometry>
          <material><ambient>0.2 0.9 0.4 1</ambient><diffuse>0.2 0.9 0.4 1</diffuse><emissive>0.1 0.5 0.2 1</emissive></material></visual>
      </link>
    </model>"""


def pedestal_sdf(name, x, y, color):
    r, gg, b = _hex_rgb(color)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x:.4f} {y:.4f} 0 0 0 0</pose>
      <link name="link">
        <visual name="plate"><pose>0 0 0.001 0 0 0</pose><geometry><box><size>0.42 0.42 0.002</size></box></geometry>
          <material><ambient>{r * 0.6:.2f} {gg * 0.6:.2f} {b * 0.6:.2f} 1</ambient><diffuse>{r * 0.6:.2f} {gg * 0.6:.2f} {b * 0.6:.2f} 1</diffuse></material></visual>
        <visual name="ring"><pose>0 0 0.0015 0 0 0</pose><geometry><cylinder><radius>1.3</radius><length>0.001</length></cylinder></geometry>
          <material><ambient>0.45 0.50 0.48 1</ambient><diffuse>0.45 0.50 0.48 1</diffuse></material></visual>
      </link>
    </model>"""


def box_sdf(name, pose, size, mass, color):
    sx, sy, sz = size
    r, gg, b = _hex_rgb(color)
    ixx = mass * (sy * sy + sz * sz) / 12
    iyy = mass * (sx * sx + sz * sz) / 12
    izz = mass * (sx * sx + sy * sy) / 12
    return f"""
    <model name="{name}">
      <pose>{pose[0]:.3f} {pose[1]:.3f} {pose[2]:.3f} 0 0 0</pose>
      <link name="link">
        <inertial><mass>{mass}</mass><inertia><ixx>{ixx:.6f}</ixx><iyy>{iyy:.6f}</iyy><izz>{izz:.6f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <material><ambient>0.72 0.55 0.36 1</ambient><diffuse>0.72 0.55 0.36 1</diffuse></material></visual>
        <visual name="tape"><pose>0 0 {sz / 2 + 0.0005:.4f} 0 0 0</pose><geometry><box><size>{sx * 0.98:.3f} 0.03 0.001</size></box></geometry>
          <material><ambient>{r:.2f} {gg:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {gg:.2f} {b:.2f} 1</diffuse></material></visual>
      </link>
    </model>"""


def cells_sdf(cfg):
    prod = cfg['production']
    out = []
    for i, (lane, cell) in enumerate(cfg['cells'].items()):
        g = conveyor_geometry(cell, prod)
        out.append(conveyor_sdf(f'conveyor_{lane}', g, prod['conveyor_top'], cell['color']))
        out.append(machine_sdf(f'machine_{lane}', g, cell['color']))
        out.append(pedestal_sdf(f'pedestal_{lane}', cell['arm'][0], cell['arm'][1], cell['color']))
        for k in range(1, prod['pool_per_lane'] + 1):
            out.append(box_sdf(box_name(lane, k), storage_pose(i, k), prod['box']['size'], prod['box']['mass'], cell['color']))
    t = cfg.get('truck_robot')
    if t:
        out.append(pedestal_sdf(f'pedestal_{t["name"]}', t['arm'][0], t['arm'][1], t['color']))
    return '\n'.join(out)


def pallet_joints_sdf(cfg):
    """Weld points pallet -> box: a placed box is welded to the pallet (like stretch wrap), so the physics
    engine no longer solves the contacts of a 32-box stack (that alone dropped the sim to RTF 0.15)."""
    out = []
    for lane in cfg['cells']:
        for k in range(1, cfg['production']['pool_per_lane'] + 1):
            b = box_name(lane, k)
            out.append(f'''
      <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
        <parent_link>link</parent_link><child_model>{b}</child_model><child_link>link</child_link>
        <attach_topic>/pallet/{b}/attach</attach_topic><detach_topic>/pallet/{b}/detach</detach_topic>
        <output_topic>/pallet/{b}/state</output_topic>
      </plugin>''')
    return ''.join(out)


def build_world(template_text, cfg):
    marker = '<!-- @@CELLS@@ -->'
    if marker not in template_text:
        raise ValueError('world template has no @@CELLS@@ marker')
    uri = '<uri>model://pallet</uri>'
    if uri not in template_text:
        raise ValueError('world template has no pallet include')
    text = template_text.replace(uri, uri + pallet_joints_sdf(cfg))
    return text.replace(marker, cells_sdf(cfg))
