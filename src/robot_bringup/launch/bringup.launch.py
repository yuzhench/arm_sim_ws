from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch.substitutions import PathJoinSubstitution 
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription


import os
import yaml


import xacro



def launch_setup(context, *args, **kwargs):
    #use simulation or the real robot 
    use_simulation = LaunchConfiguration("simulation_mode").perform(context)


    moveit_pkg = "movit_robot_arm_sim"
    arm_hw_pkg = "arm_sim_pkg"
    demo_pkg = "traj_demo"

    moveit_share = get_package_share_directory (moveit_pkg)
    arm_hw_share = get_package_share_directory (arm_hw_pkg)
    demo_share = get_package_share_directory (demo_pkg)


    #load the xacro file 
    xacro_file = Path(arm_hw_share) / 'urdf' / 'simple_arm.xacro'

    #transfer to urdf file 
    robot_description_content = xacro.process_file(
        str(xacro_file),
        mappings={
            "simulation_mode": use_simulation
        }
    ).toprettyxml(indent='  ')

    robot_description = {'robot_description': robot_description_content}


    #motor_controller yaml file 
    controllers_yaml = PathJoinSubstitution([
        arm_hw_share, 'config', 'controllers.yaml'
    ])


    # rviz configuration 
    rviz_cfg = PathJoinSubstitution([
        arm_hw_share, 'rviz', 'arm_view.rviz'
    ])



    #start the ros2 controller node 
    control_node = Node(
        package = "controller_manager", 
        executable = "ros2_control_node",
        parameters = [
            ParameterFile(controllers_yaml,  allow_substs=True),
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen"
    )

    #robot state publisher 
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",  
    )


    #Rviz2 node 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2", 
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}, robot_description],

    )


    #joint state broadcaster 
    # spawner 是 controller_manager 包里的一个小工具节点。
    # 它的作用是 请求 controller_manager 加载并启动某个控制器。
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster", 
            "--controller-manager", 
            "/controller_manager"
        ],
        output='screen',
    )



    #jpint position controller 
    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            'joint_position_controller',
            '--controller-manager',
            '/controller_manager',
        ],
        output = 'screen',
    )


    #launch for the moveit 
    move_group_launch = IncludeLaunchDescription (
        #tell current launch file where can find this embeded launch file 
         
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                moveit_share, 'launch', 'move_group.launch.py'
            ])
        ),
        #if we need to add more launch arguments 
        launch_arguments={
            #currently it is empty
        }.items()
    )

    #prepare the parameters for the demo node 
    srdf_path  = os.path.join(moveit_share, 'config', 'five_joint_arm.srdf')
    kin_yaml   = os.path.join(moveit_share, 'config', 'kinematics.yaml')
    jl_yaml    = os.path.join(moveit_share, 'config', 'joint_limits.yaml')


    with open(srdf_path, 'r') as f:
        srdf_string = f.read()
    with open(kin_yaml, 'r') as f:
        robot_description_kinematics = yaml.safe_load(f)
    with open(jl_yaml, 'r') as f:
        robot_description_planning = yaml.safe_load(f)

    # launch for the demo node 
    demo_node = Node(
        package=demo_pkg,
        executable='dynamic_send_target',
        name='dynamic_send_target',
        output='screen',
        parameters= [
            robot_description,
            {'robot_description_semantic': srdf_string},
            {'robot_description_kinematics': robot_description_kinematics},
            {'robot_description_planning': robot_description_planning},
        ]
    )

    return[
        control_node,
        robot_state_pub,
        joint_position_controller_spawner,
        joint_state_broadcaster_spawner,
        move_group_launch,
        demo_node,
        rviz_node,
    ]



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "simulation_mode",
            default_value="true",  # 默认走假硬件；想默认真机就改为 "false"
            description="Use fake hardware if true; real hardware if false."
        ),
        OpaqueFunction(function=launch_setup),
    ])