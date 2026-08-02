"""
grip_bench.launch.py — the fast rig for fixing the grasp.

Brings up ONLY what is needed to answer "does the gripper hold the box?":

  t=0   Gazebo + robot + controllers, robot spawned already docked at the table
  t=8   MoveIt move_group
  t=20  grip_bench — N trials of  place box between fingers → close → lift → verify

Deliberately NOT started:
  * Nav2               — the robot never drives
  * box_pose_estimator — the box position is set exactly, not perceived
  * grasp_attacher / smart_grasp — the whole point is to test the REAL physics grasp
                    (libgazebo_grasp_plugin). Set use_physics_grasp:=false to bring the
                    legacy kinematic weld back for an A/B comparison.

Each trial teleports the box to sit exactly between the finger faces, so centring
error is zero by construction and any failure is a property of the grip itself.
~12 s per trial versus ~210 s for a full pipeline run.

Usage:
    ros2 launch limo_car grip_bench.launch.py
    ros2 launch limo_car grip_bench.launch.py loops:=20 close_sec:=2.0
    ros2 launch limo_car grip_bench.launch.py use_physics_grasp:=false   # A/B vs weld

Live tuning while it runs (applies from the next trial — no relaunch):
    ros2 param set /grip_bench close_sec   2.0
    ros2 param set /grip_bench close_floor -0.30
    ros2 param set /grip_bench backoff     false
    ros2 param set /grip_bench box_off_y   0.005

Results are written to src/data/gripbench_<timestamp>.csv.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg    = get_package_share_directory('limo_car')
    moveit = get_package_share_directory('limo_cobot_moveit_config')

    loops             = LaunchConfiguration('loops',             default='5')
    close_sec         = LaunchConfiguration('close_sec',         default='4.0')
    use_gzclient      = LaunchConfiguration('use_gzclient',      default='true')
    use_rviz          = LaunchConfiguration('use_rviz',          default='false')
    use_physics_grasp = LaunchConfiguration('use_physics_grasp', default='true')
    capture_pose      = LaunchConfiguration('capture_pose',      default='false')
    dist              = LaunchConfiguration('dist',              default='0.24')
    lift_m            = LaunchConfiguration('lift_m',            default='0.06')
    run_bench         = LaunchConfiguration('run_bench',         default='true')
    use_perception    = LaunchConfiguration('use_perception',    default='false')
    wiggle            = LaunchConfiguration('wiggle',            default='false')

    args = [
        DeclareLaunchArgument(
            'loops', default_value='5',
            description='Number of grasp trials to run. Each one resets BOTH the robot pose and the box position first, so every trial starts identical.'),
        DeclareLaunchArgument(
            'close_sec', default_value='4.0',
            description='Gripper close duration (s). Slower = gentler contact.'),
        DeclareLaunchArgument(
            'use_gzclient', default_value='true',
            description='Gazebo GUI. Keep it on to watch the fingers — and turn on '
                        'View > Collisions to see the shapes physics actually uses.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            description='RViz. Off by default; the bench is watched in Gazebo.'),
        DeclareLaunchArgument(
            'dist', default_value='0.24',
            description='Distance from the robot to the box (m). Sets BOTH where the '
                        'robot is placed and where the arm aims, so they always agree. '
                        '0.24 is the measured arm limit; the camera needs ~0.35 to see '
                        'the whole box. Change it live with '
                        '`ros2 param set /grip_bench pose_x <d>` and `robot_y <4.0+d>`.'),
        DeclareLaunchArgument(
            'use_perception', default_value='false',
            description='true = also start box_pose_estimator, so /box_pose shows what '
                        'the CAMERA thinks the box position is. The bench itself does '
                        'not need it (it sets the box position exactly), but it is the '
                        'only way to watch perception succeed at 0.35 m and fail at '
                        '0.24 m, which is the blind-zone problem made visible.'),
        DeclareLaunchArgument(
            'run_bench', default_value='true',
            description='false = bring up ONLY the simulation and MoveIt, and do not '
                        'start the bench node. Use this for the interactive session: '
                        'run `ros2 run limo_car grip_bench --ros-args -p interactive:=true` '
                        'in a second terminal so it has a real keyboard attached.'),
        DeclareLaunchArgument(
            'lift_m', default_value='0.06',
            description='How far to lift after gripping (m). Short on purpose: the 213 mm '
                        'wrist+gripper stack must hang below the elbow, so a tall lift '
                        'pushes the elbow past its limit. 0.10 fails at a 0.15 m table.'),
        DeclareLaunchArgument(
            'capture_pose', default_value='false',
            description='true = move to the grasp point once, print the six arm joint '
                        'angles for mycobot_ros2_control.xacro, then exit. Used to '
                        'calibrate the arm SPAWN pose so it is born at the table.'),
        DeclareLaunchArgument(
            'wiggle', default_value='false',
            description='true = after each lift, roll and swing the wrist while holding '
                        'the box so you can SEE it rotate with the hand. This is the '
                        'eye test that the real fixed joint couples orientation, which '
                        'the old kinematic weld never did.'),
        DeclareLaunchArgument(
            'use_physics_grasp', default_value='true',
            description='true = test the real fixed-joint grasp plugin (the point of '
                        'this bench). false = start grasp_attacher + smart_grasp and '
                        'test the legacy kinematic weld instead, for an A/B.'),
    ]

    # ── 1. Gazebo — robot pre-docked. The box sits at world y=4.0; spawning the robot
    #    at y=4.24 puts the box at base_link x=0.24, i.e. exactly the dock distance,
    #    with no driving required.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'spawn_y':      '4.24',
            'use_rviz':     use_rviz,
            'use_gzclient': use_gzclient,
            'drive_mode':   'diff',
        }.items())

    # ── 2. MoveIt — delayed so the controllers are up first.
    moveit_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, 'launch', 'moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'use_rviz':     'false',
            }.items())])

    # ── 3. Legacy weld nodes — ONLY for the A/B comparison. See the launch arg.
    legacy_weld = TimerAction(
        period=15.0,
        condition=UnlessCondition(use_physics_grasp),
        actions=[
            Node(package='limo_car', executable='grasp_attacher',
                 name='grasp_attacher', output='screen',
                 parameters=[{'use_sim_time': True}]),
            Node(package='limo_car', executable='smart_grasp',
                 name='smart_grasp', output='screen',
                 parameters=[{'use_sim_time': True}]),
        ])

    # ── 3b. Perception — optional, purely so the camera's opinion can be watched.
    perception = TimerAction(
        period=12.0,
        condition=IfCondition(use_perception),
        actions=[Node(
            package='limo_car', executable='box_pose_estimator',
            name='box_pose_estimator', output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 4. The bench itself — move_group must be ready first.
    bench = TimerAction(
        period=20.0,
        condition=IfCondition(run_bench),
        actions=[Node(
            package='limo_car',
            executable='grip_bench',
            name='grip_bench',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'loops':     ParameterValue(loops,     value_type=int),
                'close_sec': ParameterValue(close_sec, value_type=float),
                'capture_pose': ParameterValue(capture_pose, value_type=bool),
                # dist drives both the robot placement and the arm target so they can
                # never disagree: box sits at world y=4.0, so robot_y = 4.0 + dist.
                'pose_x':  ParameterValue(dist, value_type=float),
                'robot_y': ParameterValue(
                    PythonExpression(['4.0 + ', dist]), value_type=float),
                'wiggle': ParameterValue(wiggle, value_type=bool),
                'lift_m': ParameterValue(lift_m, value_type=float),
            }])])

    return LaunchDescription(
        args + [gazebo, moveit_launch, legacy_weld, perception, bench])
