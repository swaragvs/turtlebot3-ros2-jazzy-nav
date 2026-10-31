import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AnySubstitution,
    EqualsSubstitution,
    IfElseSubstitution,
    LaunchConfiguration,
)


def generate_launch_description():
    package_share = get_package_share_directory('antropi_tb3_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_tb3_sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    default_world = os.path.join(
        nav2_tb3_sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'
    )
    custom_world = os.path.join(package_share, 'worlds', 'maze_world.sdf')
    models_path = os.path.join(package_share, 'models')

    append_gz_resource = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path,
    )
    append_gazebo_resource = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path,
    )

    use_custom_maze_arg = DeclareLaunchArgument(
        'use_custom_maze',
        default_value='false',
        description='Load the custom maze world when true',
    )
    use_custom_maze = LaunchConfiguration('use_custom_maze')

    use_custom_map_arg = DeclareLaunchArgument(
        'use_custom_map',
        default_value='false',
        description='Load the custom maze_map.yaml when true; otherwise load the stock TurtleBot3 sandbox map',
    )
    use_custom_map = LaunchConfiguration('use_custom_map')

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Enable SLAM live mapping instead of loading a static map',
    )
    slam = LaunchConfiguration('slam')
    normalized_slam = IfElseSubstitution(
        condition=EqualsSubstitution(slam, 'true'),
        if_value='True',
        else_value='False',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo server without the graphical Gazebo client',
    )
    headless = LaunchConfiguration('headless')
    normalized_headless = IfElseSubstitution(
        condition=EqualsSubstitution(headless, 'true'),
        if_value='True',
        else_value='False',
    )

    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='-0.075')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='-1.975')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.01')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='1.5708')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    chosen_world = IfElseSubstitution(
        condition=EqualsSubstitution(use_custom_maze, 'true'),
        if_value=custom_world,
        else_value=default_world,
    )

    custom_map = os.path.join(package_share, 'maps', 'maze_map.yaml')
    stock_map = os.path.join(nav2_bringup_dir, 'maps', 'tb3_sandbox.yaml')
    chosen_map = IfElseSubstitution(
        condition=AnySubstitution(
            EqualsSubstitution(use_custom_maze, 'true'),
            EqualsSubstitution(use_custom_map, 'true'),
        ),
        if_value=custom_map,
        else_value=stock_map,
    )

    custom_params = os.path.join(package_share, 'config', 'nav2_params.yaml')
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle')

    tb3_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')
        ),
        launch_arguments={
            'headless': normalized_headless,
            'use_sim_time': 'True',
            'world': chosen_world,
            'slam': normalized_slam,
            'map': chosen_map,
            'params_file': custom_params,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose,
            'yaw': yaw,
        }.items(),
    )

    return LaunchDescription([
        append_gz_resource,
        append_gazebo_resource,
        set_tb3_model,
        use_custom_maze_arg,
        use_custom_map_arg,
        slam_arg,
        headless_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        yaw_arg,
        tb3_sim,
    ])
