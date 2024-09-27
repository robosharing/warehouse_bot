# Симуляция робота и склада в Gazebo

## Установка ROS2 с зависимостями

Следуйте инструкциям, чтобы установить ROS2 с необходимыми зависимостями, как указано в [install.txt](https://github.com/robosharing/warehouse_bot/blob/main/install.txt).

#### Установка ROS2 и RMF:

```bash
mkdir -p ware_ws/src && cd $_
git clone https://github.com/robosharing/warehouse_bot.git . -b develop-rmf
cd Livox-SDK2 && rm -rf build && mkdir build && cd build
cmake .. && make -j && sudo make install
cd ~/ware_ws/src/livox_ros_driver2 && source /opt/ros/humble/setup.sh
./build.sh humble  # игнорируем предупреждения
cd ~/ware_ws && colcon build  # до исчезновения предупреждений
source /opt/ros/humble/setup.bash && source install/local_setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
```

#### Настройка окружения

```bash
cd ~/ware_ws/
source /opt/ros/humble/setup.bash && source install/local_setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r -y
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/lib/cmake/CycloneDDS' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ware_ws/src/multi_robots_rmf_nav2/models' >> ~/.bashrc
```
#### Установка и настройка rmf_web:

```bash
curl -fsSL https://get.pnpm.io/install.sh | bash -
pnpm env use --global 20
pip3 install pipenv
sudo apt install python3-venv

cd ware_ws/src/rmf-web
pnpm install
```

#### Если CMake все еще не находит CycloneDDS, попробуйте явно указать путь в переменной CycloneDDS_DIR:

```bash
export CycloneDDS_DIR=/usr/local/lib/cmake/CycloneDDS
source ~/.bashrc
```

##### Перезагрузка компьютера после выполнения вышеуказанных команд

## Запуск робота в мире склада:

```bash
ros2 launch warehouse_bot launch_sim.launch.py
```

#### Управление через клавиатуру:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Навигация:

```bash
ros2 launch warehouse_bot nav2.launch.py use_sim_time:=true
ros2 launch warehouse_bot launch_sim.launch.py
ros2 launch warehouse_bot localization_launch.py
```

## Запуск множества роботов с RMF и Nav2:
#### Настройка точек спавна в RMF:

```bash
traffic-editor

Путь к карте:
~/ware_ws/src/rmf/rmf_sim/map/ware.building.yaml
```

#### Обновление координат спавна из RMF и запуск множества роботов с Nav2 и RMF:

```bash
ros2 launch multi_robots_rmf_nav2 sim.launch.py 
ros2 launch rmf_sim warehouse_sim.launch.xml headless:=false
```

#### Управление конкретным роботом:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/<robot_name>/cmd_vel
```

#### Отправка цели для RMF:

```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -F v1 -R robot1 -p r1 --use_sim_time
```

    -F — имя флота.
    -R — имя робота.
    -p — конечная цель.

## Запуск RMF в веб-интерфейсе:
#### Запуск веб-интерфейса RMF:

```bash
cd ware_ws/src/rmf-web/packages/dashboard
pnpm start
```

#### Запуск роботов:

```bash
ros2 launch multi_robots_rmf_nav2 sim.launch.py
ros2 launch rmf_sim warehouse_sim.launch.xml server_uri:="ws://localhost:8000/_internal"
```

#### Доступ к веб-интерфейсу:

#### Откройте браузер и перейдите по адресу: http://localhost:3000/robots

#### На данный момент задача отправки целей через веб-интерфейс RMF в разработке, поэтому цели можно отправлять через ноду:

```bash
ros2 run rmf_demos_tasks dispatch_go_to_place -F v1 -R robot1 -p r1 --use_sim_time
```