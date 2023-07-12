# simulation-races
In this small project I want to show the usage of localization, mapping and sensor fusion in simulated car race.

### Installation

Install [ROS1 Noetic](http://wiki.ros.org/noetic/Installation).

Install additional ROS packages:
```
sudo apt-get install ros-noetic-ros-control ros-noetic-gazebo-ros-control ros-noetic-ros-controllers ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-amcl ros-noetic-gmapping ros-noetic-ira-laser-tools
```
Clone this repository with all submodules which were forked from mit-racecar[^1] and RoboCar 1/10[^2].
```
git clone --recurse-submodules https://github.com/CatUnderTheLeaf/simulation-races.git
```

### Usage

```
# go to race_ws/
# build and source code
catkin_make
source devel/setup.bash
 
# launch a racetrack wwith a car
roslaunch races simulation.launch

# in another terminal
# example how to make a car move
rostopic pub -r 6 /drive ackermann_msgs/AckermannDriveStamped '{header: auto, drive: {steering_angle: 0.0, speed: 0.5} }'

# example how to make a car stop
rostopic pub -r 6 /drive ackermann_msgs/AckermannDriveStamped '{header: auto, drive: {steering_angle: 0.0, speed: 0.0} }'
```


My sincere thanks to these open-source projects:
[^1]: [mit-racecar](https://github.com/mit-racecar)
[^2]: [RoboCar 1/10](https://robots.ros.org/robocar-1-10/)
