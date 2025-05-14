## Сборка
```
colcon build
source install/setup.bash
```
## Запуск
```
ros2 launch robot_tppo robot.launch.py
```

## TODO:
- Добавить lidar
- Добавить камеру
- Написать robot.py для отправки команд
- Добавить топики в launch файл из команды (для GZ-ROS2-Bridge):
```shell
ros2 run ros_gz_bridge parameter_bridge
/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
/camera@sensor_msgs/msg/Camera@gz.msgs.camera
/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```
## Описание робота 
Описание робота находится в urdf/robot.urdf.xacro
## Директории
```shell
├── README.md
├── robot-gazebo
│   ├── building_robot.sdf
│   └── robot.sdf
├── robot_tppo                          # Сам робот
│   ├── CMakeLists.txt                  # Нужен для сборки
│   ├── config                          # Контроллеры
│   │   └── diff_drive_controllers.yaml # Описание контроллеров
│   ├── launch                          # Запуск робота
│   │   └── robot.launch.py             # Файл запуска робота в RViZ и в Gazebo
│   ├── package.xml                     # Нужен для сборки
│   ├── rviz                            # rviz конфигурация для RViZ
│   │   └── urdf.rviz                   # Файл конфигурации, нужен для RViZ
│   └── urdf                            # Описание робота
│       └── robot.urdf.xacro            # xacro описание робота 
├── Schemes                             # хз для чего это
│   └── Schemes here.txt
└── scripts                             # Скрипты для проверки работы топиков             
    └── send_msg_cmd_vel.sh             # Для отправки сообщении на топик cmd_vel
```
### Документация по gz_ros2_control (смотреть примеры файлов)
https://github.com/ros-controls/gz_ros2_control/blob/jazzy/doc/index.rst
### Документация для камеры и лидара (тут туториал)
https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
