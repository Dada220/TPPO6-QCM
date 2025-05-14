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
/scripts - папка содержащая скрипты для отправки сообщении
