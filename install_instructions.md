# setup ubuntu 18

## install ros-melodic
1. `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
2. `sudo apt install curl # if you haven't already installed curl`
3. `curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`
4. `sudo apt update`
5. `sudo apt install ros-melodic-desktop-full`
6. source ros in any shell `echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`

This part didn't work perfectly and caused missing moveit dependencies

7. `sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
8. `sudo apt install python-rosdep`
9. `sudo rosdep init`
10. `rosdep update` 

Other install commands to fix some random moveit errors that appeared while building the ros package

This straight up installs moveit

11. `sudo apt install ros-melodic-moveit`
12. `sudo apt-get install ros-$ROS_DISTRO-moveit-visual-tools`

## install gazebo

1. ``sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'``
2. `wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -`
3. `sudo apt update && sudo apt upgrade`
4. `sudo apt-get install python-catkin-tools`


## setup workspace
1. `source /opt/ros/melodic/setup.bash`
2. `mkdir -p ~/ariac_ws/src`
3. `cd ~/ariac_ws`
4. `catkin init`
5. `cd src`
6. `git clone https://github.com/silentkarmi/ariac_ws.git`

The ariac repo is cloned inside the ariac directory inside source, get rid of the ariac directory. Put its contents in the src directory

7. `sudo apt-get install python-rosdep`
8. `sudo rosdep init`
9. `rosdep update`
10. `cd ~/ariac_ws`
11. `rosdep install --from-paths ./src --ignore-packages-from-source -y`
12. `$ sudo apt install ros-melodic-ros-control* ros-melodic-control* ros-melodic-gazebo-ros-control*`


## build
1. `cd ~/ariac_ws`
2. `catkin build`





### General Starting Point
https://github.com/usnistgov/ARIAC
