HILAS -  Hardware in the loop architecture for simulation and task testing 
========

Developed during KUKA challenge 2013. 

### Dependencies

* Ubuntu 14.04
* youBot firmware 1.48 & youbot_driver v0.96
* ROS (indigo) installed and setup (www.ros.org)
* rtt_ros_integration (corresponding to indigo) installed and setup. 
* [v-rep 3.1.3](http://www.coppeliarobotics.com) 

# ROS and Orocos setup #
Install ROS Indigo 
```
#!bash
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall
```
then build and compile a fresh catkin workspace
```
#!bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```
now install Orocos through the [rtt_ros_integration](https://github.com/orocos/rtt_ros_integration)
```
#!bash
export OROCOS_TARGET=gnulinux
mkdir -p ~/catkin_ws/underlay_isolated/src/orocos
cd ~/catkin_ws/underlay_isolated
git clone --recursive git://gitorious.org/orocos-toolchain/orocos_toolchain.git -b toolchain-2.7 src/orocos/orocos_toolchain
catkin_make_isolated --install
source install_isolated/setup.sh

mkdir -p ~/catkin_ws/underlay/src
cd ~/catkin_ws/underlay
git clone https://github.com/orocos/rtt_ros_integration.git src/rtt_ros_integration
catkin_make
source devel/setup.sh
```
## orocos_kdl ##
HILAS needs KDL library too, so it's necessary to install it. For first check if this dependencies are satisfied: *Eigen2*, *Sip 4.7.9 and python for the python bindings*, *>= Cmake 2.6*
then install orocos_kdl in an isolated workspace.
```
#!bash
cd ~/catkin_ws/underlay_isolated
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
catkin_make_isolated --install
source install_isolated/setup.sh
```
# HILAS setup#
Clone our repository to the ./src folder inside the catkin workspace and then follow the further steps before run catkin_make on the workspace.

```
#!bash
git clone https://yourusername@bitbucket.org/altairlab/hilas.git
```

## youBot driver ##
Follow these steps for install catkin youbot-driver.
```
#!bash
git clone https://github.com/youbot/youbot_driver/tree/hydro-devel
sudo apt-get install libcppunit-dev
catkin_make
cd src/hilas_indigo/robots/youBot/youbot_driver
mkdir lib
ln -s ~/catkin_ws/devel/lib/libYouBotDriver.so lib/libYouBotDriver.so
```
the driver **must** be cloned in the /robots/youBot folder.

## Simulator ##
The simulator included in hilas is vrep provided by Coppeliarobotics. Download the leatest version from their website [http://www.coppeliarobotics.com](http://www.coppeliarobotics.com/downloads.html), then rename the vrep folder in V-REP and place it in the /simulator folder, then install joystick-driver
```
#!bash
sudo apt-get install ros-indigo-joystick-drivers 
```
for a more extended explanation visit this guide: [http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm](http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm)

### Usage examples

Open at least 3 terminals:

* t3: `$ roscore &`
* t1: `$ cd /path/to/workspace/hilas/deployment`
* t2: `$ cd /path/to/workspace/hilas/simulator/V-REP && ./vrep.sh`
* t1: `$ cd /path/to/workspace/hilas/deployment/run.sh deployer_oodl.lua`