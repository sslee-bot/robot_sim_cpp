from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument('cart_mass', default_value='0.5',
                          description='Mass of cart'),
    DeclareLaunchArgument('pendulum_mass', default_value='0.2',
                          description='Mass of pendulum'),
    DeclareLaunchArgument('friction_coefficient', default_value='0.1',
                          description='Friction coefficient between cart and ground'),
    DeclareLaunchArgument('cart_pendulum_center_distance', default_value='0.3',
                          description='Center distance between cart and pendulum'),
    DeclareLaunchArgument('mass_moment_inertia', default_value='0.006',
                          description='Mass moment inertia of pendulum'),

    DeclareLaunchArgument('x', default_value='1.0',
                          description='Initial x position of cart'),
    DeclareLaunchArgument('gui', default_value='true', choices=[
                          'true', 'false'], description='Whether to use gui'),
    DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution(
        [FindPackageShare('ss_gazebo'), 'worlds', 'empty.world']
    ), description='World file path'),
]


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH',
                            default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory('ss_description')).
            parent.resolve())])

    cart_mass = LaunchConfiguration('cart_mass')
    pendulum_mass = LaunchConfiguration('pendulum_mass')
    friction_coefficient = LaunchConfiguration('friction_coefficient')
    cart_pendulum_center_distance = LaunchConfiguration(
        'cart_pendulum_center_distance')
    mass_moment_inertia = LaunchConfiguration('mass_moment_inertia')
    x = LaunchConfiguration('x')
    gui = LaunchConfiguration('gui')
    # world_path = PathJoinSubstitution(
    #     [FindPackageShare('ss_gazebo'), 'worlds', 'empty.world']
    # )
    world_path = LaunchConfiguration('world_path')

    # Get urdf via xacro
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('ss_description'), 'urdf',
             'inverted_pendulum.urdf.xacro']
        ),
        ' ', 'cart_mass:=', cart_mass, ' pendulum_mass:=', pendulum_mass,
        ' friction_coefficient:=', friction_coefficient,
        ' length_to_COM:=', cart_pendulum_center_distance,
        ' mass_moment_inertia:=', mass_moment_inertia,
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

    controller_node = Node(package='ss_gazebo',
                           executable='inverted_pendulum_LQR_node',
                           parameters=[{
                               'cart_mass': cart_mass,
                               'pendulum_mass': pendulum_mass,
                               'friction_coefficient': friction_coefficient,
                               'cart_pendulum_center_distance': cart_pendulum_center_distance,
                               'mass_moment_inertia': mass_moment_inertia,
                           }],
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
        arguments=['-entity', 'inverted_pendulum', '-topic',
                   'robot_description', '-x', x, '-y' '0'],
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
