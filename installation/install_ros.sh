# Updating sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Downloading ROS keys
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# ROS installation
sudo apt update
sudo apt upgrade
sudo apt install ros-noetic-desktop-full

# Updating bashrc file to automathically load ROS
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Installinguseful dependencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential

# Initializing rosdep
sudo rosdep init
rosdep update
catkin config  --extend /opt/ros/noetic