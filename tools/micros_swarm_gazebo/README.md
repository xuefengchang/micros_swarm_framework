### micros_swarm_gazebo configuration
This package depends on hector_quadrotor, following the steps below to configure:

* clone the hector_quadrotor package to the catkin workspace
```
git clone https://github.com/tu-darmstadt-ros-pkg/hector_quadrotor.git
```

* install dependent packages
```
sudo apt-get install ros-kinetic-hector-pose-estimation
sudo apt-get install ros-kinetic-hardware-interface
sudo apt-get install ros-kinetic-controller-interface
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-hector-gazebo-plugins
sudo apt-get install ros-kinetic-hector-sensors-description
```

* make
```
cd catkin_ws
catkin_make
'''

