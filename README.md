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
### Инерциальный датчик (IMU)
В rqt открыть Topic и выбрать топик /imu. Запустить движение робота и смотреть изменения датчиков(раскрывать все вкладки)

## TODO:
- [x] Добавить лидар
- [x] Добавить камеру
    - [x] Добавлена идентификация кодов Aruco  
- [x] Добавить инерциальный датчик
- [x] Добавить возможность управления роботом с помощью кнопок (в виде приложения)
- [x] Добавить вывод данных о компонентах робота (теперь работает в графическом окне)
- [x] Добавить графический интерфейс ввода команд робота и локации
- [ ] Добавить ультразвуковой датчик расстояния
- [ ] Добавить инфракрасный датчик расстояния
- [ ] Добавить дальномер
- [ ] Добавить гироскоп
- [x] Написать robot.py для отправки команд напрямую
- [x] Добавить топики в launch файл из команды (для GZ-ROS2-Bridge):
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
├── robot-gazebo                               # SDF файлы симуляции
│   ├── building_robot.sdf
│   └── robot.sdf
├── robot_tppo                                 # Сам робот
│   ├── CMakeLists.txt                         # Нужен для сборки
│   ├── config                                 # Контроллеры
│   │   └── diff_drive_controllers.yaml        # Описание контроллеров
│   ├── launch                                 # Запуск робота
│   │   └── robot.launch.py                    # Файл запуска робота в RViZ и в Gazebo
│   ├── models                                 # Папка моделей
│   │   └── arucotag                           # Модель Aruco маркера
│   │       ├── arucotag_0.png                 # Aruco маркер 4x4 c id: 0
│   │       ├── arucotag.png                   # Aruco маркер 4x4 c id: 44
│   │       ├── model.config                   # Конфиг модели
│   │       └── model.sdf                      # Сама модель маркера
│   ├── package.xml                            # Нужен для сборки
│   ├── rviz                                   # rviz конфигурация для RViZ
│   │   └── urdf.rviz                          # Файл конфигурации, нужен для RViZ
│   ├── scripts                                # Файлы взаимодействия с роботом
│   │   ├── gui                                # Виджеты для приложения
│   │   │   ├── nested_table.py                # Таблица для отображения данных компонент робота
│   │   │   └── control_frame.py               # Панель кнопок управления роботом
│   │   ├── app.py                             # Графическое окно вывода данных компонент и управления роботом
│   │   ├── commands.py                        # Файл команд
│   │   ├── robot_data_collector.py            # Класс ComponentDataCollector для сбора данных компонентов
│   │   ├── message_broker.py                  # Класс MessageBroker
│   │   └── robot.py                           # Класс Robot для передвижения
│   ├── urdf                                   # Описание робота 
│   │   ├── camera.urdf.xacro                  # xacro описание камеры 
│   │   ├── core.urdf.xacro                    # xacro описание колёс и платформы робота
│   │   ├── inertial_measuring_unit.urdf.xacro # xacro описание инерциального датчика
│   │   ├── lidar.urdf.xacro                   # xacro лидара 
│   │   └── robot.urdf.xacro                   # xacro робота с diff контроллером 
│   └── worlds                                 # папка локации
│       ├── aruco.sdf                          # локация с Aruco маркером
│       └── colorful_scene.sdf                 # локация с розовой плоскостю, синим небом и большим зелёным кубом
├── Schemes                                    # Без понятия зачем это
│   └── Schemes here.txt
└── scripts                                    # Скрипты для проверки работы топиков             
    ├── send_msg_cmd_vel.sh                    # Для отправки сообщении на топик cmd_vel
    └── text.sh                                # Для проверки движения робота назад
```

## Источники 
### Документация по gz_ros2_control
https://github.com/ros-controls/gz_ros2_control/blob/jazzy/doc/index.rst
### Документация для камеры и лидара 
https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors
### 
https://github.com/adoodevv/diff_drive_robot
### Документация датчиков Gazebo
https://github.com/gazebosim/gz-sensors
