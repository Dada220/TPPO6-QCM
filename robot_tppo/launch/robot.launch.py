import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    pkg_share = get_package_share_directory('robot_tppo')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    controller_params_file = os.path.join(pkg_share, 'config', 'diff_drive_controllers.yaml')

    # 1) Генерируем Substitution для xacro -> urdf
    robot_description_substitution = Command(['xacro ', xacro_file])
    # 2) Явно указываем, что это строка
    robot_description = ParameterValue(robot_description_substitution, value_type=str)
    # 3) Готовим словарь
    robot_description_param = {'robot_description': robot_description}

    # --- Узел robot_state_publisher ---
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]  # <-- передаём dict
    )

    # --- Узел ROS2 Control (controller_manager) ---
    # Removed because of dropped support for robot_description in ros2_control
    # link: https://robotics.stackexchange.com/questions/114404/waiting-for-data-on-robot-description-topic-to-finish-initialization-i-tried
    #ros2_control_node = Node(
    #    package='controller_manager',
    #    executable='ros2_control_node',
    #    output='screen',
    #    parameters=[
    #        #           robot_description_param,  # словарь (robot_description)
    #        controller_params_file    # путь к YAML
    #    ]
    #)

    # Спавним контроллеры
    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',  
            '--param-file', 
            controller_params_file,
            '--controller-ros-args',
            '-r /diff_drive_controller/cmd_vel:=/cmd_vel',
        ],
        output='screen',
    )

    # (Опционально) узел joint_state_publisher_gui
    # Для дифф. робота обычно не нужен, но если хотите видеть слайдеры, оставьте
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # --- RViz ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # --- Запуск GZ Sim с пустым миром ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'use_sim_time': 'true'
            }.items()
    )

    # --- Спавн робота в GZ Sim ---
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_tppo',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Bridge для преобразования сообщений между ROS и GZ
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/TwistStamped@gz.msgs.TwistStamped',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',  # To supress messages that GZ doesn't understand what time it is 
        ],
        output='screen',
    )
 
    # Собираем всё в LaunchDescription
    return LaunchDescription([        
        rsp_node,
        #ros2_control_node,
        joint_state_spawner,
        diff_drive_spawner,
        #joint_state_publisher_node,
        rviz_node,
        gz_sim,
        gz_bridge,
        spawn_entity,
    ])
