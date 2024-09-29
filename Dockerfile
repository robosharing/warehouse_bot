# Базовый образ с Ubuntu 22.04 и ROS2 Humble
FROM osrf/ros:humble-desktop

# Обновление и установка необходимых зависимостей
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    python3-pip \
    python3-colcon-mixin \
    ros-dev-tools \
    python3-ament-package \
    python3-rosdep \
    ros-humble-ament-cmake \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-typesupport-cpp \
    ros-humble-rosidl-default-generators \
    software-properties-common \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-camera-info-manager \
    ros-humble-gazebo-plugins \
    ros-humble-joint-state-publisher \
    ros-humble-rqt-robot-steering \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-cyclonedds \
    ros-humble-rmf-dev \
    pipenv \    
    libapr1-dev \
    libaprutil1-dev \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Установка дополнительных Python-пакетов
RUN python3 -m pip install flask-socketio fastapi uvicorn datamodel_code_generator


# Копирование исходного кода вашего рабочего пространства внутрь контейнера
COPY . /ware_ws/src

# Установка Livox SDK с добавлением путей к библиотекам
RUN cd /ware_ws/src/Livox-SDK2 && rm -rf build && mkdir build && cd build && \
    cmake .. && make -j && make install && \
    export LIVOX_INTERFACES_INCLUDE_DIRECTORIES=/ware_ws/src/Livox-SDK2/include

# Сборка пакета livox_ros_driver2 с добавлением переменной окружения
RUN cd /ware_ws/src/livox_ros_driver2 && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
    export LIVOX_INTERFACES_INCLUDE_DIRECTORIES=/ware_ws/src/Livox-SDK2/include && ./build.sh humble"

# Установка переменных окружения
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /ware_ws/install/local_setup.bash' >> ~/.bashrc && \
    echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/usr/local/lib/cmake/CycloneDDS' >> ~/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/ware_ws/src/multi_robots_rmf_nav2/models' >> ~/.bashrc



# Сборка всего рабочего пространства с активацией ROS2 окружения
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && cd /ware_ws && colcon build"

# Обновление зависимостей и их проверка перед сборкой
RUN rosdep update && \
    rosdep install --from-paths /ware_ws/src --ignore-src -r -y
    
# Запуск симуляции и робота
RUN /bin/bash