from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def get_robot_description(pkg_name, xacro_file):
    """处理xacro文件并返回URDF字符串"""
    pkg_share = get_package_share_directory(pkg_name)
    xacro_path = os.path.join(pkg_share, xacro_file)
    return Command(['xacro ', xacro_path])

def generate_launch_description():
    # Foxy兼容的MoveIt启动文件（带夹爪版本）
    
    pkg_name = 'piper_with_gripper_moveit'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 使用xacro处理URDF
    robot_description_content = get_robot_description(pkg_name, 'config/piper.urdf.xacro')
    robot_description = {'robot_description': robot_description_content}
    
    robot_description_semantic_content = load_file(pkg_name, 'config/piper.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}
    
    # 加载运动学配置
    kinematics_yaml = load_yaml(pkg_name, 'config/kinematics.yaml')
    
    # 加载关节限制
    joint_limits_yaml = load_yaml(pkg_name, 'config/joint_limits.yaml')
    
    # 加载控制器配置
    controllers_yaml = load_yaml(pkg_name, 'config/moveit_controllers.yaml')
    
    # Planning配置 - 使用OMPL
    planning_plugin = 'ompl_interface/OMPLPlanner'
    
    # Trajectory execution配置
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Planning scene monitor配置
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # MoveGroup参数
    move_group_params = [
        robot_description,
        robot_description_semantic,
        kinematics_yaml,
        planning_scene_monitor_parameters,
        trajectory_execution,
        {'planning_plugin': planning_plugin},
        {'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints'},
        {'start_state_max_bounds_error': 0.1},
    ]
    
    if joint_limits_yaml:
        move_group_params.append(joint_limits_yaml)
    
    if controllers_yaml:
        move_group_params.append(controllers_yaml)
    
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
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
