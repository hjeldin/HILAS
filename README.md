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
~~~
sudo apt-get install ros-indigo-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt-get install python-rosinstall ruby ruby-dev libncurses5-dev libncurses5 liblua5.1.0-dev
sudo gem install rake
~~~
download lua socket library and compile

~~~
wget http://files.luaforge.net/releases/luasocket/luasocket/luasocket-2.0.2/luasocket-2.0.2.tar.gz
tar -xvzf luasocket-2.0.2.tar.gz
cd luasocket-2.0.2
export CPATH=$CPATH:/usr/include/lua5.1
make
sudo make install
echo "export LUA_PATH=$LUA_PATH:/usr/share/lua/5.1/?.lua" >> ~/.bashrc
echo "export LUA_CPATH=$LUA_CPATH:/usr/lib/lua/5.1/?.so" >> ~/.bashrc
~~~
then build and compile a fresh catkin workspace

~~~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
~~~

now install Orocos through the [rtt_ros_integration](https://github.com/orocos/rtt_ros_integration)

~~~
export OROCOS_TARGET=gnulinux
mkdir -p ~/catkin_ws/underlay_isolated/src/orocos
cd ~/catkin_ws/underlay_isolated
git clone --recursive git://git.gitorious.org/orocos-toolchain/orocos_toolchain.git -b toolchain-2.7 src/orocos/orocos_toolchain
catkin_make_isolated --install
cd ~/catkin_ws/underlay_isolated/install_isolated/lib/ruby/1.9.1/x86_64-linux/orogen/templates/typekit
touch CATKIN_IGNORE
source ~/catkin_ws/underlay_isolated/install_isolated/setup.sh

mkdir -p ~/catkin_ws/underlay/src
cd ~/catkin_ws/underlay
git clone https://github.com/orocos/rtt_ros_integration.git src/rtt_ros_integration
catkin_make
source devel/setup.sh
~~~

## orocos_kdl ##
HILAS needs KDL library too, so it's necessary to install it. For first check if this dependencies are satisfied: *Eigen2*, *Sip 4.7.9 and python for the python bindings*, *>= Cmake 2.6*
then install orocos_kdl in an isolated workspace.

~~~
echo "source ~/catkin_ws/underlay_isolated/install_isolated/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/underlay/devel/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
cd ~/catkin_ws/underlay_isolated
git clone https://github.com/orocos/orocos_kinematics_dynamics.git src/orocos_kinematics_dynamics
catkin_make_isolated --install
source install_isolated/setup.sh
~~~

# HILAS setup#
Clone our repository to the ./src folder inside the catkin workspace and then follow run catkin_make on the workspace.  
Be sure to replace yourusername in the following 
~~~
git clone --recursive -b indigo https://yourusername@bitbucket.org/altairlab/hilas.git
sudo apt-get install libcppunit-dev
sudo apt-get install ros-indigo-joystick-drivers
~~~

## Simulator ##
The simulator included in hilas is vrep provided by Coppelia Robotics. Download the leatest version from their website [http://www.coppeliarobotics.com](http://www.coppeliarobotics.com/downloads.html), then rename the vrep folder in V-REP and place it in the /simulator folder, then install joystick-driver.
For a more extended explanation visit this guide: [http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm](http://www.coppeliarobotics.com/helpFiles/en/rosTutorialHydro.htm)

~~~
cd ~/catkin_ws
echo "export YOUBOTDIR=~/catkin_ws/src/hilas/robots/youBot/youbot_driver" >> ~/.bashrc
echo "export HILAS_HOME=~/catkin_ws/src" >> ~/.bashrc
mkdir -p ~/catkin_ws/src/hilas/robots/youBot/youbot_driver/lib
touch ~/catkin_ws/src/hilas/robots/youBot/youbot_driver/lib/libYouBotDriver.so
catkin_make
~~~

## youBot driver ##
After the workspace compilation follow these steps to setup youbot-driver for catkin.

~~~
cd src/hilas/robots/youBot/youbot_driver
mkdir lib
rm lib/libYouBotDriver.so
ln -s ~/catkin_ws/devel/lib/libYouBotDriver.so lib/libYouBotDriver.so
~~~

### Usage examples

**WIP**