## Сборка
```
colcon build
source install/setup.bash
```
## Запуск
```
ros2 launch robot_tppo robot.launch.py
```
## Проверка работоспособности
### Движение из ros2 в Gazebo и RViZ
Запустить send_msg_cmd_vel.sh в директории scripts
### Камера
В Gazebo открыть Image Display и выбрать топик /camera/image (три точки в правом верхнем углу)

В RViZ добавить Camera и в топик добавить /camera/image 
### Лидар
В Gazebo открыть Visualize Lidar и выбрать топик /scan (возможно надо будет нажать кнопу retry в Gazebo)
### Лидар
В rqt открыть Topic и выбрать топик /imu. Запустить движение робота и смотреть изменения датчиков(раскрывать все вкладки)

В RViZ добавить LazerScan и в топик добавить /scan 
## TODO:
- [x] Добавить лидар
- [x] Добавить камеру
- [x] Добавить инерциальный датчик
- [ ] Добавить ультразвуковой датчик расстояния
- [ ] Добавить инфракрасный датчик расстояния
- [ ] Добавить дальномер
- [ ] Добавить гироскоп
- [x] Написать robot.py для отправки команд напрямую
    - [ ] Доработать чтобы движение могло осуществляется на основе данных с сенсоров
- [x] Добавить топики в launch файл из команды (для GZ-ROS2-Bridge):
```shell
ros2 run ros_gz_bridge parameter_bridge
/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan
/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
/camera@sensor_msgs/msg/Camera@gz.msgs.camera
/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock
```
## Текущие проблемы
- Есть проблема когда один или два контроллера могут не активироваться. Ссылка на одно из обсуждении: https://github.com/ros-controls/gz_ros2_control/issues/421
    - Вроде как исправил в launch файле выставить timeout больше чем 5 секунд
## Описание робота 
Описание робота находится в urdf/robot.urdf.xacro
## Директории
```shell
├── README.md                            
├── robot-gazebo                               # SDF файлы симуляции
│   ├── building_robot.sdf
│   └── robot.sdf
├── robot_tppo                                 # Сам робот
│   ├── CMakeLists.txt                         # Нужен для сборки
│   ├── config                                 # Контроллеры
│   │   └── diff_drive_controllers.yaml        # Описание контроллеров
│   ├── launch                                 # Запуск робота
│   │   └── robot.launch.py                    # Файл запуска робота в RViZ и в Gazebo
│   ├── package.xml                            # Нужен для сборки
│   ├── rviz                                   # rviz конфигурация для RViZ
│   │   └── urdf.rviz                          # Файл конфигурации, нужен для RViZ
│   ├── scripts
│   │   ├── commands.py                        # файл команд
│   │   ├── __pycache__
│   │   │   └── robot.cpython-312.pyc
│   │   └── robot.py                           # класс робота для передвижения
│   ├── urdf                                   # Описание робота 
│   │   ├── camera.urdf.xacro                  # xacro описание камеры 
│   │   ├── core.urdf.xacro                    # xacro описание колёс и платформы робота
│   │   ├── inertial_measuring_unit.urdf.xacro # xacro описание инерциального датчика
│   │   └── lidar.urdf.xacro                   # xacro лидара 
│   │   └── robot.urdf.xacro                   # xacro робота с diff контроллером 
│   └── worlds                                 # папка локации
│       ├── colorful_scene.sdf                 # локация с розовой плоскостю, синим небом и большим зелёным кубом
│       ├── green_cube.sdf                     # локация с маленьким зелёным кубом (из туториала)
│       └── home.sdf                           # нерабочая локация
├── Schemes                                    # хз зачем это
│   └── Schemes here.txt
└── scripts                                    # Скрипты для проверки работы топиков             
    └── send_msg_cmd_vel.sh                    # Для отправки сообщении на топик cmd_vel
```
### Документация по gz_ros2_control (смотреть примеры файлов)
https://github.com/ros-controls/gz_ros2_control/blob/jazzy/doc/index.rst
### Документация для камеры и лидара (тут какой-то туториал, не работает почему-то)
https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
### Тут вроде как лидар есть
https://github.com/adoodevv/diff_drive_robot
### Документация датчиков Gazebo
https://github.com/gazebosim/gz-sensors
