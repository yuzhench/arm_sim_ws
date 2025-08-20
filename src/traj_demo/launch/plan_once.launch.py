# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from launch.substitutions import Command, PathJoinSubstitution
# from launch_ros.parameter_descriptions import ParameterValue
# import os

# def generate_launch_description():
#     # 用你用 MSA 生成的 MoveIt 配置包名
#     moveit_pkg = 'movit_robot_arm_sim'
#     share = get_package_share_directory(moveit_pkg)

#     # 文件路径（按你的包里实际文件名改）
#     urdf_xacro = PathJoinSubstitution([share, 'config', 'five_joint_arm.urdf.xacro'])
#     srdf_path  = os.path.join(share, 'config', 'five_joint_arm.srdf')
#     kin_yaml   = os.path.join(share, 'config', 'kinematics.yaml')
#     jl_yaml    = os.path.join(share, 'config', 'joint_limits.yaml')

#     # URDF → robot_description（用 xacro 展开）
#     robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)
#     # SRDF → robot_description_semantic（读成字符串）
#     srdf_string = open(srdf_path, 'r').read()

#     return LaunchDescription([
#         # 可选：把 move_group 后端也一起起（推荐，一条命令全起）
#         IncludeLaunchDescription(PythonLaunchDescriptionSource(
#             os.path.join(share, 'launch', 'move_group.launch.py')
#         )),

#         # 你的 demo 节点 + 注入参数
#         Node(
#             package='traj_demo',
#             executable='plan_once',
#             name='plan_once',
#             output='screen',
#             parameters=[
#                 {'robot_description': robot_description},
#                 {'robot_description_semantic': srdf_string},
#                 kin_yaml,     # → robot_description_kinematics
#                 jl_yaml,      # → robot_description_planning
#             ],
#         ),
#     ])


# traj_demo/launch/plan_once.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
import os, yaml

def generate_launch_description():
    moveit_pkg = 'movit_robot_arm_sim'
    share = get_package_share_directory(moveit_pkg)

    urdf_xacro = PathJoinSubstitution([share, 'config', 'five_joint_arm.urdf.xacro'])
    srdf_path  = os.path.join(share, 'config', 'five_joint_arm.srdf')
    kin_yaml   = os.path.join(share, 'config', 'kinematics.yaml')
    jl_yaml    = os.path.join(share, 'config', 'joint_limits.yaml')

    # 1) URDF → robot_description（xacro 展开；先忽略 stderr，防止弃用/包含警告中断）
    robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)


    # 2) SRDF → robot_description_semantic
    with open(srdf_path, 'r') as f:
        srdf_string = f.read()

    # 3) YAML → dict → 注入
    with open(kin_yaml, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)
    with open(jl_yaml, 'r') as f:
        robot_description_planning = yaml.safe_load(f)

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'move_group.launch.py')
        )),
        Node(
            package='traj_demo',
            executable='plan_once',
            name='plan_once',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': srdf_string},
                {'robot_description_kinematics': robot_description_kinematics},
                {'robot_description_planning': robot_description_planning},
            ],
        ),
    ])
