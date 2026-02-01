
#Önce gerekli olan paketlerin kurumu yapılacaktır.

sudo apt-get install cmake curl git gitk git-gui python3-pip screen apt-transport-https -y

sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
        
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install python3-catkin-tools -y
sudo pip install dronekit
sudo pip3 install -U catkin_tools

#Ros ve gazebo kurulumu için

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update

sudo apt install ros-noetic-desktop-full ros-noetic-web-video-server -y

export ROS_PACKAGE_PATH=/home/$USER/estuiha_gazebo/worlds:/opt/ros/hydro/share:/opt/ros/hydro/stacks:$ROS_PACKAGE_PATH

#Web server için

cd
mkdir catkin
cd catkin
mkdir src
cd src
git clone https://github.com/sfalexrog/async_web_server_cpp.git -b noetic-devel
git clone https://github.com/RobotWebTools/web_video_server.git
cd ..
catkin build
sudo apt install python3-roslaunch -y

#ardupilot sitl kurmak için

cd
git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot
git submodule update --init --recursive

Tools/environment_install/install-prereqs-ubuntu.sh -y

######
. ~/.profile

#Pid ayarlarının doğru çalışabilmesi için

rm ~/ardupilot/Tools/autotest/default_params/plane.parm
touch ~/ardupilot/Tools/autotest/default_params/plane.parm

#estuiha_gazebo klasörünü home yoluna çıkardıktan sonra

cd estuiha_gazebo
mkdir build
cd build
cmake ..
make -j8
sudo make install

sudo gedit ~/.bashrc

#Açılan ekrana aşağıdaki source ve export yollarını en alta ekleyin

source /usr/share/gazebo/setup.sh
source /opt/ros/noetic/setup.bash
export GAZEBO_RESOURCE_PATH=~/estuiha_gazebo/worlds:${GAZEBO_RESOURCE_PATH}
export GAZEBO_PLUGIN_PATH=~/estuiha_gazebo/build:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=~/estuiha_gazebo/models:${GAZEBO_MODEL_PATH}

# simülasyonu başlatmak için

bash ~/estuiha_gazebo/start.sh


