#!/usr/bin/env python3
"""
Foxy 兼容的 MoveIt2 Demo Launch 文件
启动 move_group, RViz2, 和虚拟机械臂
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    """加载 YAML 文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def load_file(package_name, file_path):
    """加载文件内容"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def generate_launch_description():
    """生成 launch 描述"""
    
    # 声明参数
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 包路径
    moveit_pkg_share = get_package_share_directory('piper_with_gripper_moveit')
    description_pkg_share = get_package_share_directory('piper_description')
    urdf_file_path = os.path.join(description_pkg_share, 'urdf', 'piper_description.urdf')
    
    # 直接加载 URDF 文件
    robot_description_content = load_file('piper_description', 'urdf/piper_description.urdf')
    robot_description = {'robot_description': robot_description_content}
    
    # 加载 SRDF
    robot_description_semantic_config = load_file('piper_with_gripper_moveit', 'config/piper.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}
    
    # 运动学配置
    kinematics_yaml = load_yaml('piper_with_gripper_moveit', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}
    
    # 规划配置
    ompl_planning_yaml = load_yaml('piper_with_gripper_moveit', 'config/ompl_planning.yaml')
    
    # 默认值
    planning_plugin = 'ompl_interface/OMPLPlanner'
    request_adapters = ''
    start_state_max_bounds_error = 0.1
    
    # 构建 ompl_planning_pipeline_config
    ompl_planning_pipeline_config = {}
    
    if ompl_planning_yaml:
        # 提取顶层配置
        planning_plugin = ompl_planning_yaml.get('planning_plugin', planning_plugin)
        request_adapters = ompl_planning_yaml.get('request_adapters', request_adapters)
        start_state_max_bounds_error = ompl_planning_yaml.get('start_state_max_bounds_error', start_state_max_bounds_error)
        
        # 添加每个 planner_config 的详细配置
        if 'planner_configs' in ompl_planning_yaml:
            planner_configs = ompl_planning_yaml['planner_configs']
            ompl_planning_pipeline_config['move_group.planner_configs'] = list(planner_configs.keys())
            
            # 为每个 planner 添加详细配置
            for planner_name, planner_config in planner_configs.items():
                for key, value in planner_config.items():
                    ompl_planning_pipeline_config[f'move_group.planner_configs.{planner_name}.{key}'] = value
        
        # 添加 arm 配置
        if 'arm' in ompl_planning_yaml:
            for key, value in ompl_planning_yaml['arm'].items():
                ompl_planning_pipeline_config[f'move_group.arm.{key}'] = value
        
        # 添加 gripper 配置
        if 'gripper' in ompl_planning_yaml:
            for key, value in ompl_planning_yaml['gripper'].items():
                ompl_planning_pipeline_config[f'move_group.gripper.{key}'] = value
    
    ompl_planning_pipeline_config['move_group.planning_plugin'] = planning_plugin
    ompl_planning_pipeline_config['move_group.request_adapters'] = request_adapters
    ompl_planning_pipeline_config['move_group.start_state_max_bounds_error'] = start_state_max_bounds_error
    
    # Trajectory execution 配置
    # 禁用controller管理，仅用于规划不执行
    trajectory_execution = {
        'moveit_manage_controllers': False,  # 不管理controllers，仅规划
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    moveit_controllers_yaml = load_yaml('piper_with_gripper_moveit', 'config/moveit_controllers.yaml')
    moveit_controllers = moveit_controllers_yaml if moveit_controllers_yaml else {}
    
    # Planning Scene Monitor 配置
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }
    
    # Joint limits 配置
    joint_limits_yaml = load_yaml('piper_with_gripper_moveit', 'config/joint_limits.yaml')
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}
    
    # ========================================
    # 节点定义
    # ========================================
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )
    
    # Joint State Publisher GUI (fake hardware) - 用GUI版本自动发布关节状态
    initial_joint_positions = load_yaml('piper_with_gripper_moveit', 'config/initial_positions.yaml')
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',  # 使用GUI版本
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        parameters=[
            robot_description,  # 添加robot_description参数
            initial_joint_positions if initial_joint_positions else {},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )
    
    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    
    # RViz2
    rviz_config_file = os.path.join(moveit_pkg_share, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_planning,
            robot_description_kinematics,
            ompl_planning_pipeline_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher,
        move_group_node,
        rviz_node,
    ])
