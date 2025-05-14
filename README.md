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
- Добавить топики launch из команды (для GZ-ROS2-Bridge):
```shell
ros2 run ros_gz_bridge parameter_bridge
/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
/camera@sensor_msgs/msg/Camera@gz.msgs.camera
/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```
## Описание робота 
Описание робота находится в urdf/robot.urdf.xacro
## Скрипты
```shell
├── README.md
├── robot-gazebo
│   ├── building_robot.sdf
│   └── robot.sdf
├── robot_tppo                          # Сам робот
│   ├── CMakeLists.txt
│   ├── config                          # Контроллеры
│   │   └── diff_drive_controllers.yaml
│   ├── launch                          # Запуск робота
│   │   └── robot.launch.py
│   ├── package.xml
│   ├── rviz                            # rviz конфигурация для RViZ
│   │   └── urdf.rviz
│   └── urdf                            # Описание робота
│       └── robot.urdf.xacro
├── Schemes                             # хз для чего это
│   └── Schemes here.txt
└── scripts                             # Скрипты для проверки работы топиков             
    └── send_msg_cmd_vel.sh  
```
