"""LD-60 palletizing plant: Gazebo + 4 TM12S cells + AMR + MoveIt + plant manager + dashboard.

    ros2 launch ld60_plant plant.launch.py                    # Gazebo window + dashboard
    ros2 launch ld60_plant plant.launch.py gz_gui:=false      # lighter
    ros2 launch ld60_plant plant.launch.py rviz:=true         # also RViz (MoveIt view of the arms)
Dashboard: http://localhost:8081
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ld60_plant.world_gen import build_world

RUN_DIR = os.path.expanduser('~/.ld60_logs/run')


def generate_launch_description():
    pkg = get_package_share_directory('ld60_plant')
    os.makedirs(RUN_DIR, exist_ok=True)
    cfg_path = os.path.join(pkg, 'config', 'plant.yaml')
    with open(cfg_path) as fh:
        cfg = yaml.safe_load(fh)
    gz_gui = LaunchConfiguration('gz_gui')
    rviz = LaunchConfiguration('rviz')
    web_port = LaunchConfiguration('web_port')

    # ---------- generated world (cells from plant.yaml) ----------
    with open(os.path.join(pkg, 'worlds', 'plant.sdf')) as fh:
        world_text = build_world(fh.read(), cfg)
    world = os.path.join(RUN_DIR, 'plant_world.sdf')
    with open(world, 'w') as fh:
        fh.write(world_text)

    # ---------- arm models with the real controller path ----------
    controllers = os.path.join(pkg, 'config', 'arms', 'ros2_controllers.yaml')
    urdf_paths = {}
    for name in ('plant_arms.urdf', 'plant_arms_gz.urdf'):           # (the gripper bar sdf needs no edits)
        with open(os.path.join(pkg, 'urdf', name)) as fh:
            text = fh.read().replace('@@CONTROLLERS@@', controllers)
        urdf_paths[name] = os.path.join(RUN_DIR, name)
        with open(urdf_paths[name], 'w') as fh:
            fh.write(text)
    with open(urdf_paths['plant_arms.urdf']) as fh:
        arms_urdf = fh.read()

    moveit = (
        MoveItConfigsBuilder('plant_arms', package_name='ld60_plant')
        .robot_description(file_path=urdf_paths['plant_arms.urdf'])
        .robot_description_semantic(file_path='config/arms/plant_arms.srdf')
        .robot_description_kinematics(file_path='config/arms/kinematics.yaml')
        .joint_limits(file_path='config/arms/joint_limits.yaml')
        .trajectory_execution(file_path='config/arms/moveit_controllers.yaml')
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(default_planning_pipeline='pilz_industrial_motion_planner',
                            pipelines=['pilz_industrial_motion_planner'])
        .pilz_cartesian_limits(file_path='config/arms/pilz_cartesian_limits.yaml')
        .to_moveit_configs()
    )
    sim_time = {'use_sim_time': True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        # started PAUSED (no -r): sim_boot releases the vacuum joints first, then unpauses
        launch_arguments={'gz_args': f'-s -v 1 {world}', 'on_exit_shutdown': 'true'}.items(),
    )
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g', '-v', '1', '--render-engine-gui', 'ogre',
             '--gui-config', os.path.join(pkg, 'config', 'gz_gui_plant.config')],
        output='log', condition=IfCondition(gz_gui), respawn=True, respawn_delay=3.0,
        additional_env={'GALLIUM_DRIVER': 'd3d12', 'QT_QPA_PLATFORM': 'xcb'},
    )
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='log',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='log',
               parameters=[{'robot_description': arms_urdf}, sim_time])
    lanes = [lane.lower() for lane in cfg['cells']]
    if cfg.get('truck_robot'):
        lanes.append(cfg['truck_robot']['name'].lower())
    spawners = Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=['joint_state_broadcaster'] + [f'{p}_controller' for p in lanes] +
                              ['--controller-manager-timeout', '180', '--param-file', controllers])
    move_group = Node(package='moveit_ros_move_group', executable='move_group', output='log',
                      parameters=[moveit.to_dict(), sim_time])
    rviz_node = Node(package='rviz2', executable='rviz2', output='log', condition=IfCondition(rviz),
                     additional_env={'QT_QPA_PLATFORM': 'xcb'},
                     parameters=[moveit.robot_description, moveit.robot_description_semantic,
                                 moveit.robot_description_kinematics, sim_time])
    amr = Node(package='ld60_plant', executable='amr_manager', output='screen', parameters=[{'config': cfg_path}, sim_time])
    plant = Node(package='ld60_plant', executable='plant_manager', output='screen',
                 parameters=[sim_time, {'config': cfg_path,
                                        'homes': os.path.join(pkg, 'config', 'arms', 'homes.yaml')}])
    rosbridge = Node(package='rosbridge_server', executable='rosbridge_websocket', output='log',
                     parameters=[{'port': 9090, 'address': '0.0.0.0'}])
    web = Node(package='ld60_plant', executable='web_server', output='screen',
               parameters=[{'port': web_port, 'root': os.path.join(pkg, 'web')}])

    # staged boot (see sim_boot): world -> arms -> truck gripper bar, releasing each group of joints
    bar = os.path.join(pkg, 'urdf', f'{cfg["truck_robot"]["name"].lower()}_bar.sdf') if cfg.get('truck_robot') else ''
    boot = ExecuteProcess(cmd=['python3', '-m', 'ld60_plant.sim_boot', '--config', cfg_path,
                               '--arms', urdf_paths['plant_arms_gz.urdf'], '--bar', bar], output='screen')
    after_boot = RegisterEventHandler(OnProcessExit(target_action=boot, on_exit=[spawners]))
    after_ctrl = RegisterEventHandler(OnProcessExit(target_action=spawners, on_exit=[move_group, rviz_node, plant]))

    return LaunchDescription([
        DeclareLaunchArgument('gz_gui', default_value='true', description='Show the Gazebo window'),
        DeclareLaunchArgument('rviz', default_value='false', description='Show RViz'),
        DeclareLaunchArgument('web_port', default_value='8081', description='Dashboard port'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory('tm_description'))),
        gazebo, gazebo_gui, bridge, rsp, boot, after_boot, after_ctrl,
        amr, rosbridge, web,
    ])
