1. install ROS2 with dependenses NEED ALL   (in install.txt)

2. mkdir ware_ws

3. cd ware_ws

4. mkdir src

5. cd src

6. git clone https://github.com/robosharing/warehouse_bot.git

7. cd livox_ros_driver2

8. ./build.sh humble

9. cd .. ..

10. colcon build 

11. source install/local_setup.bash

(FOR GAZEBO VISUALISATION без AMCL)

10. ros2 launch warehouse_bot launch_sim.launch.py

11. ros2 launch warehouse_bot ros2_control.launch.py

move with teleop twist keyboard:

11.  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/tricycle_controller/cmd_vel

(FOR SLAM)

12. ros2 launch slam_toolbox online_async_launch.py params_file:=./src/warehouse_bot/config/mapper_params_online_async.yaml use_sim_time:=true 

(TWIX_MUX FOR NAV2)

13. ros2 run twist_mux twist_mux --ros-args --params-file ./src/warehouse_bot/config/twist_mux.yaml -r cmd_vel_out:=/tricycle_controller/cmd_vel 

(FOR NAV2)

14. ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true





====================================================

инструкция для GAZEBO 

1. распаковываем models.zip в папку home/.gazebo (по умолчанию она скрыта, чтобы увидеть нажимаем ctrl+h)

2.заходим в каждую папку и исправляем файл .sdf -> Там меняем путь к файлу  .DAE, например -> 

<uri>file:///home/меняем тут на имя компьютера/.gazebo/models/aws_robomaker_warehouse_WallB_01/meshes/aws_robomaker_warehouse_WallB_01_collision.DAE</uri>

3. нужно исправить в каждой папке, иначе не заработает в дальнейшем

4. заходим в газебо

5. тут в вкладке insert должны появиться модельки

6. расставляем как нужно

7. сохраняем в формате .world в папку имя_рабочего_пространства/src/warehouse_bot/world


===================================================

FOR LAUNCH IN WORLD WITH AMCL NAV2

for launch in world

15. ros2 launch warehouse_bot launch_sim.launch.py world:=./src/warehouse_bot/world/t.world

for launch localization 

16. ros2 launch warehouse_bot localization_launch.py

set initial pose, and run navigation 

17. ros2 run twist_mux twist_mux --ros-args --params-file ./src/warehouse_bot/config/twist_mux.yaml -r cmd_vel_out:=/tricycle_controller/cmd_vel

18. ros2 launch warehouse_bot navigation_launch.py

