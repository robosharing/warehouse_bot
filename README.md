install ROS2 with dependenses NEED ALL (in install.txt)

make workspace and colcon build:

```bash
mkdir -p ware_ws/src
cd ./ware_ws/src
git clone https://github.com/robosharing/warehouse_bot.git .
cd ./livox_ros_driver2
source /opt/ros/humble/setup.sh
./build.sh humble
cd ./ware_ws
colcon build 
source install/local_setup.bash
```
open new terminal (we need source /opt/ros/humble/setup.bash, not source /opt/ros/humble/setup.sh)

(FOR GAZEBO VISUALISATION без AMCL)
```bash
ros2 launch warehouse_bot launch_sim.launch.py
ros2 launch warehouse_bot ros2_control.launch.py
```
move with teleop twist keyboard:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tricycle_controller/cmd_vel
```
(FOR SLAM)
```bash
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/warehouse_bot/config/mapper_params_online_async.yaml use_sim_time:=true 
```
(TWIX_MUX FOR NAV2)
```bash
ros2 run twist_mux twist_mux --ros-args --params-file ./src/warehouse_bot/config/twist_mux.yaml -r cmd_vel_out:=/tricycle_controller/cmd_vel 
```
(FOR NAV2)
```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

====================================================

инструкция для GAZEBO 

1. распаковываем models.zip в папку home/.gazebo (по умолчанию она скрыта, чтобы увидеть нажимаем ctrl+h)

2. заходим в газебо

3. тут в вкладке insert должны появиться модельки

4. расставляем как нужно

5. сохраняем в формате .world в папку имя_рабочего_пространства/src/warehouse_bot/world


===================================================

FOR LAUNCH IN WORLD WITH AMCL NAV2

for launch in world
```bash
ros2 launch warehouse_bot launch_sim.launch.py world:=./src/warehouse_bot/world/t.world
```
for launch localization 
```bash
ros2 launch warehouse_bot localization_launch.py
```
set initial pose, and run navigation 
```bash
ros2 run twist_mux twist_mux --ros-args --params-file ./src/warehouse_bot/config/twist_mux.yaml -r cmd_vel_out:=/tricycle_controller/cmd_vel
```
```bash
ros2 launch warehouse_bot navigation_launch.py
```
