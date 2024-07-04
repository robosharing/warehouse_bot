1. install ROS2 with dependenses NEED ALL (in install.txt)

2. Compile and install the Livox-SDK2: https://github.com/Livox-SDK/Livox-SDK2

3. mkdir -p ware_ws/src

4. cd ./ware_ws/src

5. git clone https://github.com/robosharing/warehouse_bot.git

6. cd ./warehouse_bot/livox_ros_driver2

7. ./build.sh humble

8. cd ../../

9. colcon build 

10. source install/local_setup.bash

(FOR GAZEBO VISUALISATION без AMCL)

11. ros2 launch warehouse_bot launch_sim.launch.py

12. ros2 launch warehouse_bot ros2_control.launch.py

move with teleop twist keyboard:

13.  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tricycle_controller/cmd_vel

(FOR SLAM)

14. ros2 launch slam_toolbox online_async_launch.py params_file:=./src/warehouse_bot/config/mapper_params_online_async.yaml use_sim_time:=true 

(TWIX_MUX FOR NAV2)

15. ros2 run twist_mux twist_mux --ros-args --params-file ./src/warehouse_bot/config/twist_mux.yaml -r cmd_vel_out:=/tricycle_controller/cmd_vel 

(FOR NAV2)

16. ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true


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

15. ros2 launch warehouse_bot launch_sim.launch.py world:=./src/warehouse_bot/world/t.world

for launch localization 

16. ros2 launch warehouse_bot localization_launch.py

set initial pose, and run navigation 

17. ros2 run twist_mux twist_mux --ros-args --params-file ./src/warehouse_bot/config/twist_mux.yaml -r cmd_vel_out:=/tricycle_controller/cmd_vel

18. ros2 launch warehouse_bot navigation_launch.py

