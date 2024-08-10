### Создаем пространство

```bash
mkdir ~/rmf_ws/src -p
cd ~/rmf_ws/src
git clone https://github.com/robosharing/rmf.git
cd ~/rmf_ws
colcon build
```
---

### Запуск мира

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos_gz_classic warehouse.launch.xml
```
---

### Посылаем команды для движения
```bash
ros2 run rmf_demos_tasks dispatch_patrol -p (начальная точка) (конечная точка) -n (количество повотрений) --use_sim_time
```
