"""Generate the 4-arm palletizing robot model + controller + MoveIt config from config/plant.yaml.

Run after changing the cell layout (needs ROS, ~/ros2_ws and ~/plant_ws sourced - xacro, tm_description,
and collisions_updater resolving the ld60_plant meshes):
    python3 ~/plant_ws/tools/gen_arms.py
Writes into src/ld60_plant:
    urdf/plant_arms.urdf          full model (RViz / MoveIt / robot_state_publisher)
    urdf/plant_arms_gz.urdf       Gazebo model: light meshes, no arm collisions, vacuum plugins
    config/arms/*.yaml + .srdf    controllers, MoveIt (kinematics, limits, Pilz, SRDF)
    config/arms/homes.yaml        home joint angles per arm (deg)
"""
import math
import os
import re
import subprocess
import sys
import tempfile

import numpy as np
import yaml

PKG = os.path.expanduser('~/plant_ws/src/ld60_plant')
sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/tm12s_sim'))
from tm12s_sim.kin_check import Chain, pose  # noqa: E402

JOINT_VEL = [2.268, 2.268, 3.665, 3.927, 3.927, 7.854]        # URDF (rad/s)
PLAN_VEL = [2.0, 2.0, 3.3, 3.5, 3.5, 7.0]                      # 10 % margin for planning
PLAN_ACC = [3.0, 3.0, 4.0, 5.0, 5.0, 6.0]


def cfg():
    with open(os.path.join(PKG, 'config', 'plant.yaml')) as fh:
        return yaml.safe_load(fh)


def all_arms(c):
    """[(name, cell, pedestal height, gripper)] - the 4 palletizing cells plus the truck robot."""
    arms = [(lane, cell, c['pedestal_height'], None) for lane, cell in c['cells'].items()]
    t = c.get('truck_robot')
    if t:
        arms.append((t['name'], t, t['pedestal_height'], t['gripper']))
    return arms


def tool_xacro(prefix, gripper):
    if gripper is None:                                     # single vacuum cup
        return f"""
  <link name="{prefix}vacuum_cup">
    <visual><origin xyz="0 0 0.01"/><geometry><box size="0.12 0.08 0.02"/></geometry><material name="vac_black"/></visual>
    <collision><origin xyz="0 0 0.01"/><geometry><box size="0.12 0.08 0.02"/></geometry></collision>
    <inertial><origin xyz="0 0 0.01"/><mass value="0.08"/><inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00002"/></inertial>
  </link>"""
    L, W = gripper['length'], gripper['width']                # multi-box bar: 4 cups along the tool X axis
    cups = ''.join(f'<visual><origin xyz="{x:.4f} 0 0.004"/><geometry><cylinder radius="0.035" length="0.008"/></geometry>'
                   f'<material name="vac_black"/></visual>' for x in (-0.2175, -0.0725, 0.0725, 0.2175))
    return f"""
  <link name="{prefix}vacuum_cup">
    <visual><origin xyz="0 0 0.014"/><geometry><box size="{L} {W} 0.012"/></geometry><material name="vac_grey"/></visual>
    {cups}
    <collision><origin xyz="0 0 0.01"/><geometry><box size="{L} {W} 0.02"/></geometry></collision>
    <inertial><origin xyz="0 0 0.01"/><mass value="1.2"/><inertia ixx="0.0008" ixy="0" ixz="0" iyy="0.034" iyz="0" izz="0.034"/></inertial>
  </link>"""


def arm_xacro(prefix, cell, h, gripper=None):
    x, y = cell['arm']
    yaw = math.radians(cell['arm_yaw'])
    return f"""
  <!-- ================= {prefix[:-1].upper()} ================= -->
  <link name="{prefix}pedestal">
    <visual><origin xyz="0 0 {h / 2}"/><geometry><cylinder radius="0.13" length="{h}"/></geometry><material name="pedestal_grey"/></visual>
    <collision><origin xyz="0 0 {h / 2}"/><geometry><cylinder radius="0.13" length="{h}"/></geometry></collision>
  </link>
  <joint name="{prefix}pedestal_joint" type="fixed">
    <parent link="world"/><child link="{prefix}pedestal"/><origin xyz="{x} {y} 0" rpy="0 0 0"/>
  </joint>
  <joint name="{prefix}mount" type="fixed">
    <parent link="{prefix}pedestal"/><child link="{prefix}base"/><origin xyz="0 0 {h}" rpy="0 0 {yaw:.6f}"/>
  </joint>
  <xacro:tm12s ns="" prefix="{prefix}" color="graywhite" format="obj" />
  <link name="{prefix}vacuum_body">
    <visual><origin xyz="0 0 0.04"/><geometry><cylinder radius="0.032" length="0.08"/></geometry><material name="vac_grey"/></visual>
    <collision><origin xyz="0 0 0.04"/><geometry><cylinder radius="0.032" length="0.08"/></geometry></collision>
    <inertial><origin xyz="0 0 0.04"/><mass value="0.4"/><inertia ixx="0.0003" ixy="0" ixz="0" iyy="0.0003" iyz="0" izz="0.0002"/></inertial>
  </link>
  <joint name="{prefix}vacuum_body_joint" type="fixed"><parent link="{prefix}flange"/><child link="{prefix}vacuum_body"/></joint>
{tool_xacro(prefix, gripper)}
  <joint name="{prefix}vacuum_cup_joint" type="fixed"><parent link="{prefix}vacuum_body"/><child link="{prefix}vacuum_cup"/><origin xyz="0 0 0.08"/></joint>
  <link name="{prefix}vacuum_tcp"/>
  <joint name="{prefix}vacuum_tcp_joint" type="fixed"><parent link="{prefix}vacuum_cup"/><child link="{prefix}vacuum_tcp"/><origin xyz="0 0 0.02"/></joint>
"""


def build_xacro(c, homes):
    arms = ''.join(arm_xacro(f'{name.lower()}_', cell, h, grip) for name, cell, h, grip in all_arms(c))
    joints = ''
    for lane, _, _, _ in all_arms(c):
        p = lane.lower() + '_'
        for i, q in enumerate(homes[lane]):
            joints += f"""
      <joint name="{p}joint_{i + 1}">
        <command_interface name="velocity"/>
        <state_interface name="position"><param name="initial_value">{math.radians(q):.6f}</param></state_interface>
        <state_interface name="velocity"/>
      </joint>"""
    return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/gen_arms.py from config/plant.yaml - do not edit by hand -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="plant_arms">
  <xacro:include filename="$(find tm_description)/xacro/macro.materials.xacro" />
  <xacro:include filename="$(find tm_description)/xacro/macro.tm12s-nominal.urdf.xacro" />
  <material name="pedestal_grey"><color rgba="0.25 0.28 0.32 1"/></material>
  <material name="vac_grey"><color rgba="0.55 0.57 0.60 1"/></material>
  <material name="vac_black"><color rgba="0.08 0.08 0.08 1"/></material>
  <link name="world"/>
{arms}
  <ros2_control name="GazeboSimSystem" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>{joints}
  </ros2_control>
  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>@@CONTROLLERS@@</parameters>
    </plugin>
  </gazebo>
</robot>
"""


def dedupe_materials(urdf):
    """The TM macro defines its materials once per arm; URDF wants each top-level material once."""
    seen = set()

    def keep(m):
        name = m.group(1)
        if name in seen:
            return ''
        seen.add(name)
        return m.group(0)
    return re.sub(r'\n\s*<material name="([^"]+)">\s*<color[^>]*/>\s*</material>', keep, urdf)


def gz_variant(urdf, c):
    """Light meshes, no collisions on the arms (MoveIt checks those), vacuum plugins per box."""
    out = re.sub(r'meshes/tm12s/visual/(tm12s-[\w-]+)\.obj', r'meshes/tm12s/collision/\1_c.stl', urdf)
    out = re.sub(r'<collision>.*?</collision>', '', out, flags=re.S)
    plugins = ''
    for lane in c['cells']:
        p = lane.lower() + '_'
        for k in range(1, c['production']['pool_per_lane'] + 1):
            box = f'box_{lane}_{k:02d}'
            plugins += f"""
  <gazebo>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>{p}link_6</parent_link><child_model>{box}</child_model><child_link>link</child_link>
      <attach_topic>/vacuum/{box}/attach</attach_topic><detach_topic>/vacuum/{box}/detach</detach_topic>
      <output_topic>/vacuum/{box}/state</output_topic><suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </gazebo>"""
    t = c.get('truck_robot')
    if t:
        # the truck gripper's own box joints live in a separate bar model (see bar_sdf): every
        # DetachableJoint attaches when its model loads, and a box held by two joints of one model
        # makes DART build "welded mirror" bodies that slow and destabilise the physics. The bar
        # itself is welded to the wrist as soon as sim_boot spawns it.
        p = t['name'].lower() + '_'
        plugins += f"""
  <gazebo>
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>{p}link_6</parent_link><child_model>{p}bar</child_model><child_link>link</child_link>
      <attach_topic>/{p}bar/attach</attach_topic><detach_topic>/{p}bar/detach</detach_topic>
      <output_topic>/{p}bar/state</output_topic><suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </gazebo>"""
    return out.replace('</robot>', plugins + '\n</robot>')


def bar_sdf(c):
    """Invisible, collision-free carrier welded to the truck robot's wrist; holds one joint per box."""
    t = c['truck_robot']
    p = t['name'].lower() + '_'
    joints = ''
    for lane in c['cells']:
        for k in range(1, c['production']['pool_per_lane'] + 1):
            box = f'box_{lane}_{k:02d}'
            joints += f"""
    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
      <parent_link>link</parent_link><child_model>{box}</child_model><child_link>link</child_link>
      <attach_topic>/tgrip/{box}/attach</attach_topic><detach_topic>/tgrip/{box}/detach</detach_topic>
      <output_topic>/tgrip/{box}/state</output_topic>
    </plugin>"""
    return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/gen_arms.py - truck gripper carrier -->
<sdf version="1.9">
  <model name="{p}bar">
    <link name="link">
      <inertial><mass>0.05</mass><inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <gravity>false</gravity>
    </link>{joints}
  </model>
</sdf>
"""


def homes_by_ik(c):
    """Home = hovering 0.28 m above the arm's own pick point (elbow up)."""
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as fh:
        single = subprocess.run(['xacro', os.path.expanduser('~/ros2_ws/src/tm12s_sim/urdf/tm12s_cell.urdf.xacro'),
                                 'sim_gz:=false'], capture_output=True, text=True, check=True).stdout
        fh.write(single)
        path = fh.name
    ch = Chain(path)
    rng = np.random.default_rng(1)
    sys.path.insert(0, os.path.join(PKG))
    from ld60_plant.world_gen import conveyor_geometry
    homes = {}
    for lane, cell in c['cells'].items():
        g = conveyor_geometry(cell, c['production'])
        bx, by = cell['arm']
        byaw = math.radians(cell['arm_yaw'])
        px, py = g['pick']
        dx, dy = px - bx, py - by
        lx, ly = math.cos(-byaw) * dx - math.sin(-byaw) * dy, math.sin(-byaw) * dx + math.cos(-byaw) * dy
        lz = c['production']['conveyor_top'] + c['production']['box']['size'][2] + 0.28 - c['pedestal_height']
        rel = cell['tcp_yaw'] - cell['arm_yaw']
        best = None
        for _ in range(60):
            seed = np.radians([math.degrees(math.atan2(ly, lx)) + rng.uniform(-30, 30), rng.uniform(-40, 20),
                               rng.uniform(60, 130), rng.uniform(-60, 60), 90, rng.uniform(-180, 180)])
            q, ok = ch.ik(pose([lx * 1000, ly * 1000, lz * 1000], (180, 0, rel)), seed, iters=300)
            if ok and q[2] > 0:
                q = [math.atan2(math.sin(v), math.cos(v)) for v in q]       # wrap to +-180
                if best is None or abs(q[1]) < abs(best[1]):
                    best = q
        if best is None:
            raise SystemExit(f'{lane}: no IK for the home pose')
        homes[lane] = [round(math.degrees(v), 2) for v in best]
    t = c.get('truck_robot')
    if t:                                     # truck robot home: between pallet and stack, high, bar along X
        bx, by = t['arm']
        byaw = math.radians(t['arm_yaw'])
        dx, dy = t['pallet'][0] + 0.45 - bx, t['pallet'][1] - by
        lx, ly = math.cos(-byaw) * dx - math.sin(-byaw) * dy, math.sin(-byaw) * dx + math.cos(-byaw) * dy
        lz = c['production']['pallet_top'] + 0.42 - t['pedestal_height']
        best = None
        for _ in range(60):
            seed = np.radians([math.degrees(math.atan2(ly, lx)) + rng.uniform(-30, 30), rng.uniform(-40, 20),
                               rng.uniform(60, 130), rng.uniform(-60, 60), 90, rng.uniform(-180, 180)])
            q, ok = ch.ik(pose([lx * 1000, ly * 1000, lz * 1000], (180, 0, -t['arm_yaw'])), seed, iters=300)
            if ok and q[2] > 0:
                q = [math.atan2(math.sin(v), math.cos(v)) for v in q]
                if best is None or abs(q[1]) < abs(best[1]):
                    best = q
        if best is None:
            raise SystemExit(f'{t["name"]}: no IK for the home pose')
        homes[t['name']] = [round(math.degrees(v), 2) for v in best]
    os.unlink(path)
    return homes


def write(rel, text):
    path = os.path.join(PKG, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        fh.write(text)
    print('wrote', rel)


def main():
    c = cfg()
    lanes = [a[0] for a in all_arms(c)]
    truck = c.get('truck_robot', {}).get('name')
    homes = homes_by_ik(c)
    for lane, q in homes.items():
        print(f'home {lane}: {q}')
    xa = build_xacro(c, homes)
    with tempfile.NamedTemporaryFile('w', suffix='.urdf.xacro', delete=False) as fh:
        fh.write(xa)
        xpath = fh.name
    urdf = subprocess.run(['xacro', xpath], capture_output=True, text=True, check=True).stdout
    os.unlink(xpath)
    urdf = dedupe_materials(urdf)
    write('urdf/plant_arms.urdf', urdf)
    write('urdf/plant_arms_gz.urdf', gz_variant(urdf, c))
    if c.get('truck_robot'):
        write(f'urdf/{c["truck_robot"]["name"].lower()}_bar.sdf', bar_sdf(c))

    # ---- controllers
    ctrl = {
        'gz_ros_control': {'ros__parameters': {'hold_joints': True}},
        'controller_manager': {'ros__parameters': {
            'update_rate': 250, 'use_sim_time': True, 'enforce_command_limits': False,
            'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'},
            **{f'{l.lower()}_controller': {'type': 'joint_trajectory_controller/JointTrajectoryController'} for l in lanes}}},
        'joint_state_broadcaster': {'ros__parameters': {'update_rate': 50}},
    }
    for l in lanes:
        p = l.lower() + '_'
        js = [f'{p}joint_{i}' for i in range(1, 7)]
        ctrl[f'{p}controller'] = {'ros__parameters': {
            'joints': js, 'command_interfaces': ['velocity'], 'state_interfaces': ['position', 'velocity'],
            'allow_partial_joints_goal': False, 'open_loop_control': False, 'state_publish_rate': 25.0,
            'action_monitor_rate': 20.0,
            'gains': {j: {'p': 12.0, 'i': 0.0, 'd': 0.0, 'i_clamp': 0.0, 'ff_velocity_scale': 1.0} for j in js},
            # the truck robot carries a 4-box bar (~5 kg on the wrist): let it settle a little longer
            'constraints': {'stopped_velocity_tolerance': 0.05, 'goal_time': 2.0 if l == truck else 1.0,
                            **{j: {'trajectory': 0.15, 'goal': 0.02 if l == truck else 0.005} for j in js}}}}
    write('config/arms/ros2_controllers.yaml', '# GENERATED by tools/gen_arms.py\n' + yaml.safe_dump(ctrl, sort_keys=False))

    # ---- MoveIt
    kin = {f'{l.lower()}': {'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
                            'kinematics_solver_search_resolution': 0.005, 'kinematics_solver_timeout': 0.05} for l in lanes}
    write('config/arms/kinematics.yaml', yaml.safe_dump(kin, sort_keys=False))
    lim = {'default_velocity_scaling_factor': 1.0, 'default_acceleration_scaling_factor': 1.0, 'joint_limits': {}}
    for l in lanes:
        for i in range(6):
            lim['joint_limits'][f'{l.lower()}_joint_{i + 1}'] = {
                'has_velocity_limits': True, 'max_velocity': PLAN_VEL[i],
                'has_acceleration_limits': True, 'max_acceleration': PLAN_ACC[i]}
    write('config/arms/joint_limits.yaml', yaml.safe_dump(lim, sort_keys=False))
    write('config/arms/pilz_cartesian_limits.yaml', yaml.safe_dump(
        {'cartesian_limits': {'max_trans_vel': 1.0, 'max_trans_acc': 2.0, 'max_trans_dec': -2.0, 'max_rot_vel': 1.57}}))
    mc = {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
          'moveit_simple_controller_manager': {'controller_names': [f'{l.lower()}_controller' for l in lanes]},
          'trajectory_execution': {'allowed_execution_duration_scaling': 1.5, 'allowed_goal_duration_margin': 2.0,
                                   'allowed_start_tolerance': 0.02, 'execution_duration_monitoring': True}}
    for l in lanes:
        mc['moveit_simple_controller_manager'][f'{l.lower()}_controller'] = {
            'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory', 'default': True,
            'joints': [f'{l.lower()}_joint_{i}' for i in range(1, 7)]}
    write('config/arms/moveit_controllers.yaml', yaml.safe_dump(mc, sort_keys=False))
    groups = ''
    for l in lanes:
        p = l.lower()
        jv = ''.join(f'    <joint name="{p}_joint_{i + 1}" value="{math.radians(q):.6f}"/>\n' for i, q in enumerate(homes[l]))
        groups += f'  <group name="{p}"><chain base_link="{p}_base" tip_link="{p}_vacuum_tcp"/></group>\n'
        groups += f'  <group_state name="home" group="{p}">\n{jv}  </group_state>\n'
    srdf = f'<?xml version="1.0"?>\n<!-- GENERATED by tools/gen_arms.py; collision pairs added by collisions_updater -->\n<robot name="plant_arms">\n{groups}  <virtual_joint name="world_virtual" type="fixed" parent_frame="world" child_link="world"/>\n</robot>\n'
    write('config/arms/plant_arms.srdf', srdf)
    write('config/arms/homes.yaml', yaml.safe_dump({'homes_deg': homes}, sort_keys=False))

    # ---- disabled collision pairs (adjacent / never / always in collision), sampled by MoveIt
    srdf_path = os.path.join(PKG, 'config/arms/plant_arms.srdf')
    r = subprocess.run(['ros2', 'run', 'moveit_setup_assistant', 'collisions_updater',
                        '--urdf', os.path.join(PKG, 'urdf/plant_arms.urdf'), '--srdf', srdf_path,
                        '--output', srdf_path, '--default', '--trials', '20000'], capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit('collisions_updater failed:\n' + r.stdout + r.stderr)
    with open(srdf_path) as fh:
        print('SRDF disabled pairs:', fh.read().count('<disable_collisions'))


if __name__ == '__main__':
    main()
