import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Ensure robot model is set
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    # Launch Nav2 + Gazebo + RViz simulation
    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'headless': 'False',
            'use_sim_time': 'True',
            'initial_pose_x': '-2.0',
            'initial_pose_y': '-0.5',
            'initial_pose_yaw': '0.0',
        }.items()
    )

    return LaunchDescription([
        set_tb3_model,
        tb3_sim
    ])
