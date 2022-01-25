import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare("robot_description").find("robot_description")
    model_file = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    rviz_config_file = os.path.join(pkg_share, "config", "display_carlos.rviz")

    with open(model_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro", " ", model_file])
#            "robot_description": robot_desc,
#            "use_tf_static": False
        }]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file]
    )
    gazebo_spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', '/robot_description', '-entity', 'carlos', '-x', '1' , '-y', '1', '-z', '0', '-R', '0' ,'-P', '0' ,'-Y', '0'],
                    output='screen')

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        gazebo_spawn_entity,
        rviz_node
    ])


