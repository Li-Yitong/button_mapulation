from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Foxy兼容的MoveIt启动文件（不使用moveit_configs_utils）
    
    pkg_share = get_package_share_directory('piper_no_gripper_moveit')
    
    # 加载配置文件
    robot_description = {'robot_description': open(os.path.join(pkg_share, 'config/piper.urdf.xacro')).read()}
    robot_description_semantic = {'robot_description_semantic': open(os.path.join(pkg_share, 'config/piper.srdf')).read()}
    kinematics_yaml = load_yaml('piper_no_gripper_moveit', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('piper_no_gripper_moveit', 'config/joint_limits.yaml')
    
    # Planning配置
    planning_yaml = load_yaml('piper_no_gripper_moveit', 'config/ompl_planning.yaml') or {'planning_plugin': 'ompl_interface/OMPLPlanner'}
    
    # MoveGroup参数
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        {'planning_plugin': planning_yaml.get('planning_plugin', 'ompl_interface/OMPLPlanner')},
        {'publish_robot_description_semantic': True},
        {'allow_trajectory_execution': True},
        {'publish_planning_scene': True},
        {'publish_geometry_updates': True},
        {'publish_state_updates': True},
        {'publish_transforms_updates': True},
    ]
    
    # MoveGroup节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_params,
    )
    
    # RViz
    rviz_config = os.path.join(pkg_share, 'config/moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[robot_description, robot_description_semantic],
    )
    
    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
