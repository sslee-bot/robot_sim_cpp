from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gz_resource_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[
        EnvironmentVariable('GAZEBO_MODEL_PATH',
                            default_value=''),
        '/usr/share/gazebo-11/models/:',
        str(Path(get_package_share_directory('ss_description')).
            parent.resolve())])

    world_path = PathJoinSubstitution(
        [FindPackageShare('ss_gazebo'), 'worlds', 'empty.world']
    )

    # Get urdf via xacro
    robot_description_command = [
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution(
            [FindPackageShare('ss_description'), 'urdf',
             'inverted_pendulum.urdf.xacro']
        ),
        # TODO: xacro arguments
        # ' ', 'cart_mass:='
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

    # TODO: Controller node
    controller_node = Node(package='ss_gazebo',
                           executable='inverted_pendulum_LQR_node',
                           # TODO: parameters
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
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        arguments=['-entity', 'inverted_pendulum', '-topic',
                   'robot_description', '-x', '0', '-y' '0'],
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(gz_resource_path)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_node)

    return ld
