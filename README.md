HILAS -  Hardware in the loop architecture for simulation and task testing 
========

Developed during KUKA challenge 2013. 

### Dependencies

* Ubuntu 12.04
* youBot firmware 1.48 & youbot_driver v0.96
* ROS (fuerte) installed and setup (www.ros.org)
* rosinstall & rosdep, if not already installed: 
* orocos_toolchain_ros (corresponding to fuerte) installed and setup. 
* [v-rep 3.1.2](http://www.coppeliarobotics.com) 

Copy paste in terminal:

`$ sudo apt-get install ros-fuerte-desktop python-rosinstall python-rosdep ros-fuerte-orocos-* ros-fuerte-rtt-* libcap2-bin lua5.1 liblua5.1-socket2`

### Building instruction

* `$ git clone git@bitbucket.org:hjeldin/hilas.git /path/to/ROS/workspace`
* `$ rosmake motion_control fbsched rFSM youbot_driver`
* Download vrep (link in Dependencies section)
* Add /path/to/vrep/programming/ros_stacks to your ROS_PACKAGE_PATH
* `$ roscd vrep_common`
* `$ mkdir build && cd build && cmake .. && make -j`
* `$ cd ../../vrep_plugin`
* Remove my symbolic links and create yours, pointing to the right V-REP folders (/programming/include and /programming/common).
* `$ mkdir build`
* `$ cd build && cmake .. && make -j`
* Modify build.sh to reflect your folder hierarchy, it should copy the newly generated /lib/libv_repExtRos.so into your V-REP main folder

### Usage examples

Open at least 3 terminals:

* t3: `$ roscore &`
* t1: `$ cd /path/to/workspace/hilas/youbot-stack/deployment`
* t2: `$ cd /path/to/vrep/ && ./vrep.sh`
* t1: `$ ./run.sh deployer_oodl.lua`