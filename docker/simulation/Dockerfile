FROM osrf/ros:noetic-desktop-full

# Gerekli sistem paketleri
RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    git \
    python3-pip \
    python3-catkin-tools \
    screen \
    wget \
    lsb-release \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# ArduPilot Kurulumu
WORKDIR /home/uav
RUN git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive \
    && USER=root ./Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && rm -rf /var/lib/apt/lists/*

# Path Ayarları
ENV PATH="/home/uav/ardupilot/Tools/autotest:${PATH}"
ENV PATH="/usr/lib/ccache:${PATH}"

# Simülasyon Dosyalarını Hazırla
WORKDIR /home/uav/estuiha_gazebo
COPY Simülasyon/iha_gazebo/iha_gazebo .

# Pluginleri Derle
RUN mkdir build && cd build && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cmake .. && make -j$(nproc)"

# ROS Video Server ve Bağımlılıkları
RUN mkdir -p /home/uav/catkin_ws/src && cd /home/uav/catkin_ws/src && \
    git clone https://github.com/sfalexrog/async_web_server_cpp.git -b noetic-devel && \
    git clone https://github.com/RobotWebTools/web_video_server.git && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd .. && catkin build"

# Ortam değişkenleri
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /home/uav/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    echo "export GAZEBO_RESOURCE_PATH=/home/uav/estuiha_gazebo/worlds:\${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc && \
    echo "export GAZEBO_PLUGIN_PATH=/home/uav/estuiha_gazebo/build:\${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/home/uav/estuiha_gazebo/models:\${GAZEBO_MODEL_PATH}" >> ~/.bashrc

EXPOSE 14550 14551 8080 11311

CMD ["/bin/bash"]
