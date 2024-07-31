Симуляция робота и склада в Gazebo

install ROS2 with dependenses NEED ALL ([in install.txt](https://github.com/robosharing/warehouse_bot/blob/main/install.txt))

make workspace and colcon build:

```bash
mkdir -p ware_ws/src
cd ~/ware_ws/src
git clone https://github.com/robosharing/warehouse_bot.git .
cd ./Livox-SDK2/
rm -rf build
mkdir build
cd build
cmake .. && make -j
sudo make install
cd ~/ware_ws/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble (там будут предупреждения, их игнорируем)
cd ~/ware_ws
colcon build (несколько раз, пока без предупреждения не исчезнут)
source /opt/ros/humble/setup.bash
source install/local_setup.bash
```

ЗАПУСК РОБОТА В МИРЕ СКЛАДА:

```bash
ros2 launch warehouse_bot launch_sim.launch.py
```

УПРАВЛЕНИЕ ЧЕРЕЗ КЛАВИАТУРУ:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

НАВИГАЦИЯ:

```bash
ros2 launch warehouse_bot navigation_launch.py use_sim_time:=true
ros2 launch warehouse_bot launch_sim.launch.py
ros2 launch warehouse_bot localization_launch.py
```

МНОГО РОБОТОВ  (ДО 10), чтобы бы было больше надо добавить в эту папку https://github.com/robosharing/warehouse_bot/tree/main/warehouse_bot/robots_config контроллеров

```bash
ros2 launch warehouse_bot milti_spawn_robots.py num_robots:=<num>
```

УПРАВЛЯТЬ КАКИМ ТО РОБОТОМ

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot_<num_robot>/cmd_vel
```
