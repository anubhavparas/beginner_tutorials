# Simple ROS Publisher Subscriber with simple ROS Service
This is a simple example to run publisher and subscriber with ros service in ROS Melodic.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
 ---
## Dependencies
- Ubuntu 18.04 (LTS)
- ROS Melodic

## Instructions to build and run the code
 - Make sure you have ROS Melodic installed in your computer. If not refer to [site](http://wiki.ros.org/melodic/Installation/Ubuntu).
 
 - Create a workspace:
 ```
 mkdir -p ~/pubsub_ws/src
 cd ~/pubsub_ws/src
 ```
 - Clone the repository into the workspace:
 ```
 git clone https://github.com/anubhavparas/beginner_tutorials.git
 ```
 - Build the workspace:
 ```
 cd ~/pubsub_ws
 catkin_make or catkin build (preferred)
 source devel/setup.bash
 ```

### Running with the launch file
- Run using launch file: This will spawn:
    - a ros service node: `/anubhavp/modify_string_service`
    - a publisher node: `/anubhavp/talker`
    - a subscriber node: `/anubhavp/listener`
```
roslaunch beginner_tutorials beginner_tutorials.launch buffer_size:=1 loop_rate:=10
# arguments are:
# buffer_size - default=1
# loop_rate - default=10
```

- To call the service:
```
rosservice call /anubhavp/modify_string_service "inputstring: 'Let's learn ROS.'"
```

### Running without the launch file
- Start ros master node in a separate terminal:
```
roscore
```

- To run the service run the following in a new terminal:
```
cd ~/pubsub_ws/
source devel/setup.bash
rosrun beginner_tutorials modify_string_service
```

- To run the publisher run the following in a new terminal:
```
cd ~/pubsub_ws/
source devel/setup.bash
rosrun beginner_tutorials talker 1 10
```

- To run the subscriber run the following in a new terminal:
```
cd ~/pubsub_ws/
source devel/setup.bash
rosrun beginner_tutorials listener
```

- To call the service:
```
rosservice call /modify_string_service "inputstring: 'Let's learn ROS.'"
```


