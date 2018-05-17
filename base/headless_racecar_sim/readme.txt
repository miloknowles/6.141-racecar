denonstration: https://www.youtube.com/watch?v=A_xZnOwjMl4


# the following is some rough installation guidelines. I figured these out while installing on a clean Linux install, so it should hit most of the main points, but you might need to do a bit of debugging along the way if things don't go exactly according to plan


ROS_WS_INSTALL_DIR=~/ros/default_ws
RANGELIBC_INSTALL_DIR=~/ros

# ------------------ install ros ------------------

# follow these instructions: http://wiki.ros.org/kinetic/Installation/Ubuntu

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# set up a default ros workspace  
mkdir -p ${ROS_WS_INSTALL_DIR}/src
cd ${ROS_WS_INSTALL_DIR}
catkin_make

# ------------------ install range_libc ------------------

cd ${RANGELIBC_INSTALL_DIR}
git clone https://github.com/kctess5/range_libc
cd range_libc/pywrapper

# if there's no CUDA available:
sudo python setup.py install

# with cuda:
sudo WITH_CUDA=ON python setup.py install

# ------------------ make sure you have a racecar installation ------------------

sudo apt-get install ros-kinetic-serial ros-kinetic-ackermann-msgs ros-kinetic-map-server ros-kinetic-joy ros-kinetic-urg-node
ros-kinetic-razor-imu-9dof

git clone https://github.com/mit-racecar/racecar
git clone https://github.com/mit-racecar/vesc
cd ${RANGELIBC_INSTALL_DIR}
catkin_make

# ------------------ get the simulator GIT repo ------------------

pip install recordclass ujson
cd ${ROS_WS_INSTALL_DIR}/src
git clone https://github.mit.edu/chwalsh/headless_racecar_sim
cd ${ROS_WS_INSTALL_DIR}
catkin_make

# ------------------ test the simulator ------------------

source ${ROS_WS_INSTALL_DIR}/devel/setup.bash

# start the map server
roslaunch headless_simulator map_server.launch

# start teleop as normal - make sure your joypad dongle is connected to your computer
roslaunch racecar teleop.launch

# start the headless simulator
roslaunch headless_simulator simulate.launch 

# start rviz as normal using the provided config
rviz base.rviz

# if you're not using the provided rviz config, you can get the interesting bits like this...
# add a RobotModel instance from the Add > By Display Type menu
# add the headless simulator tools by clicking the blue + button on the tool bar and selecting the tools under headless_sim_rviz
#    if you don't see those tools, make sure you did the source ${ROS_WS_INSTALL_DIR}/devel/setup.bash step
# add odometry, laser scan from Add > By Topic menu

# after all that is done, if things worked, you should see a little blue racecar model which you can drive around with your joypad. you should see laser scan working as expected. Try moving around the racecar with your mouse using the three custom RVIZ tools (drag, place, rotate)