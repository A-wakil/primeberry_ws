import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='primeberry_two' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp_small.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control':'true'}.items()
    )

    robot_description = "./../description/em_3905.urdf.xacro"
    controller_params_file = "./../config/ackermann_controllers.yaml"

    # Controller manager node
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description},
                    controller_params_file]
    )

    # Delayed start of controller manager
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    # Ackerman drive controller spawner
    ackerman_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackerman_drive_controller"],
    )

    delayed_ackerman_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackerman_drive_spawner],
        )
    )

    # Joint state broadcaster spawner
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )
    
  
    
   
    # Launch them all!
    return LaunchDescription([
        rsp,
        # delayed_controller_manager,
        # delayed_ackerman_drive_spawner,
        # delayed_joint_broad_spawner
    ])