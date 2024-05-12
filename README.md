1. install ROS2    (in install.txt)

2. mkdir ware_ws

3. cd ware_ws

4. mkdir src

5. cd src

6. git clone

7. cd .. ..

8. colcon build --symlink-install

9. source install/setup.bash

(FOR GAZEBO VISUALISATION)

10. ros2 launch warehouse_bot launch_sim.launch.py 

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


for launch in world

15. ros2 warehouse_bot launch_sim.launch.py world:=./src/warehouse_bot/world/test.world
