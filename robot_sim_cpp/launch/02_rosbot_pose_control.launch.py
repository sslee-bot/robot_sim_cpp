from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    # For simulation
    DeclareLaunchArgument('gui', default_value='true',
                          choices=['true', 'false']),
    DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution(
        [FindPackageShare('robot_sim_cpp'), 'worlds', 'empty.world']
    ), description='World file path'),

    # For controller
    DeclareLaunchArgument('gamma_1', default_value='0.3',),
    DeclareLaunchArgument('gamma_2', default_value='3.0',),
    DeclareLaunchArgument('h', default_value='1.0',),
    DeclareLaunchArgument('k', default_value='0.5',),
    DeclareLaunchArgument('mu', default_value='1.0',),
]


def generate_launch_description():

    # Launch arguments
    gui = LaunchConfiguration('gui')
    world_path = LaunchConfiguration('world_path')
    gamma_1 = LaunchConfiguration('gamma_1')
    gamma_2 = LaunchConfiguration('gamma_2')
    h = LaunchConfiguration('h')
    k = LaunchConfiguration('k')
    mu = LaunchConfiguration('mu')

    # Gazebo resources
    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH',
                            default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory('ss_description')).parent.resolve()),
        ':', str(Path(get_package_share_directory('rosbot_description')).parent.resolve())])

    # Get urdf via xacro
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('rosbot_description'),
             'urdf', 'rosbot.urdf.xacro'],
        ),
        ' ', 'use_sim:=true',
        ' ', 'simulation_engine:=gazebo-classic',
    ]

    robot_description_content = ParameterValue(
        Command(robot_description_command),
        value_type=str
    )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }],
                                      )

    # Set controller
    controller_node = Node(package='ss_gazebo', executable='wheeled_mobile_robot_pose_control_node',
                           parameters=[{
                               'robot_model': 'rosbot',
                               'period': 0.02,
                               'model_state_topic': '/gazebo/model_states',
                               'target_state_topic': '/target_pose',
                               'control_topic': '/cmd_vel',
                               'gamma_1': gamma_1,
                               'gamma_2': gamma_2,
                                   'h': h,
                                   'k': k,
                                   'mu': mu,
                           }],
                           output='screen',
                           emulate_tty=True,
                           #    prefix=['gdb -ex=r --args'],
                           prefix="xterm -e",
                           )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '--verbose',
             world_path
             ],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=IfCondition(gui),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-entity', 'rosbot', '-topic',
                   'robot_description',
                   # '-x', x, '-y' '0'
                   ],
        output='screen',
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_node)

    return ld
