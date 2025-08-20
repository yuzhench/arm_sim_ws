import os
from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import xacro
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    pkg_share = FindPackageShare('arm_sim_pkg').find('arm_sim_pkg')

    # 1) xacro 转 URDF
    xacro_file = Path(get_package_share_directory('arm_sim_pkg')) / "urdf" / "simple_arm.xacro"
    robot_description_content = xacro.process_file(
        str(xacro_file),
        mappings={
        }
    ).toprettyxml(indent='  ')

    
    robot_description = {'robot_description': robot_description_content}



    # 2) 控制器配置文件路径
    controllers_yaml = PathJoinSubstitution([
        FindPackageShare('arm_sim_pkg'), 'config', 'controllers.yaml'
    ])

    # 3) RViz 配置
    rviz_cfg = PathJoinSubstitution([
        FindPackageShare('arm_sim_pkg'), 'rviz', 'view.rviz'
    ])

    # # A) 启动 ros2_control 框架节点
    # control_node = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     parameters=[controllers_yaml],
    #     output='screen'
    # )

    # # B) Joint State Broadcaster
    # joint_state_broadcaster_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )

    # # C) Joint Position Controller - 等待 joint_state_broadcaster 启动后再启动
    # joint_position_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=['joint_position_controller', '--controller-manager', '/controller_manager'],
    #     output='screen',
    # )


    # A) 启动 ros2_control 框架节点
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            ParameterFile(controllers_yaml, allow_substs=True),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output='screen'
    )

    # ------------------------------------------
    # D) Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # # E) Joint State Publisher GUI --> only used for simulation
    # joint_state_pub_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui',
    #     output='screen'
    # )

    # F) RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}, robot_description]
    )
    # ------------------------------------------





    # B) Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen',
    )

    
    # delayed_jsb = TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])



    # C) Joint Position Controller
    joint_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_position_controller',
            '--controller-manager', '/controller_manager',
            # '--controller-manager-timeout', '120',
            
        ],
        output='screen',
    )


    # joint_group_position_controller_spawner = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'joint_group_position_controller',   # ← 这里改成新的控制器名
    #         '--controller-manager', '/controller_manager',
    #     ],
    #     output='screen',
    # )


    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_position_controller_spawner,
    #         on_exit=[joint_state_broadcaster_spawner],
    #     )
    # )



    # # 添加延迟启动逻辑
    # delay_position_controller_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[joint_position_controller_spawner],
    #     )
    # )

     

    return LaunchDescription([
        control_node,
        robot_state_pub,
        joint_position_controller_spawner,
        # joint_group_position_controller_spawner,
        joint_state_broadcaster_spawner,
        rviz_node,
        # delay_rviz_after_joint_state_broadcaster_spawner, 
        # delay_joint_state_broadcaster_after_robot_controller_spawner,
    ])





