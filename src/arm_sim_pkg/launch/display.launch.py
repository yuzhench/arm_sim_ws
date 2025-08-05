# display.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 找到 xacro 文件
    xacro_file = PathJoinSubstitution([
        FindPackageShare('arm_sim_pkg'),
        'urdf', 'simple_arm.xacro'
    ])
    # 在运行时调用 xacro，将结果作为 robot_description
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    return LaunchDescription([
        # 1) 发布 robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',  # 静态可执行名
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # 2) Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',  # 静态可执行名
            name='joint_state_publisher_gui',
            output='screen',
        ),
        # 3) RViz2 可视化
        Node(
            package='rviz2',
            executable='rviz2',  # 静态可执行名
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                PathJoinSubstitution([
                  FindPackageShare('arm_sim_pkg'),
                  'rviz', 'view.rviz'
                ])
            ],
        ),
    ])


# from launch import LaunchDescription
# from launch.actions import LogInfo

# def generate_launch_description():
#     return LaunchDescription([
#         LogInfo(msg='[arm_sim_pkg] display.launch.py has been found and is running!'),
#     ])