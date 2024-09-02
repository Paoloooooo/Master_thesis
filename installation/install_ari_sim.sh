# Creating ari workspace
mkdir ./ari_ws
cd ./ari_ws

# Installing ari dependencies
sudo apt install python-is-python3
curl https://raw.githubusercontent.com/pal-robotics/ari_tutorials/master/ari_public-noetic.rosinstall > ari_dependencies.txt
rosinstall src /opt/ros/noetic ari_dependencies.txt
rm ari_dependencies.txt

# Rosdep packages installation
sudo rosdep init
rosdep update

rosdep install --from-paths src --ignore-src --rosdistro noetic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3 librealsense2-dev librealsense2-dkms hey5_transmissions ydlidar_ros_driver python-pyside orocos_kdl" -y -r

# Building the workspace
source /opt/ros/noetic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0