# Simple ROS Publisher Subscriber
This is a simple example to run publisher and subscriber in ROS Melodic.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
 ---

 ## Instructions to run the code
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

- Start ros master node in a separate terminal:
```
roscore
```

- To run the publisher run the following in a new terminal:
```
cd ~/pubsub_ws/
source devel/setup.bash
rosrun beginner_tutorials talker
```

- To run the subscriber run the following in a new terminal:
```
cd ~/pubsub_ws/
source devel/setup.bash
rosrun beginner_tutorials listener
```


