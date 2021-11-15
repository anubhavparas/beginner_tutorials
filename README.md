# Simple ROS Publisher Subscriber with simple ROS Service
This is a simple example to run publisher and subscriber with ros service in ROS Melodic.

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
 ---
## Functionalities
- Simple ROS publisher-subscriber
- Simple ROS Service 
- TF Broadcaster
- Gtest and ROSTest
- rosbag

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
    # is_record_bag - default=false
    # bagfile - default=recorded_topics.bag
    ```

- To enable rosbag recording. By default it is disabled. 
    - This will store the rosbag recording in a file `home/<username>/.ros/recorded_topics.bag`
        ```
        roslaunch beginner_tutorials beginner_tutorials.launch is_record_bag:=true
        ```

- To call the service:
    ```
    rosservice call /anubhavp/modify_string_service "inputstring: 'Let us learn ROS.'"
    ```

- `/talker` node also broadcasts tf frame information from `/world` to `/talk` frame. [Example](results/tf_echo_world_to_talk.png). 
    - To check the frame information run
        ```
        rosrun tf tf_echo world talk
        ```
    - For visualizing the tree of frames being broadcasted over ROS. [Example](results/rqt_tf_tree_world_to_talk.png).
        ```
        rosrun rqt_tf_tree rqt_tf_tree 
        ```
    - `view_frames` creates a diagram of the frames being broadcast by tf over ROS. [Example](results/view_frames_result.pdf).
        ```
         rosrun tf view_frames
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
    rosservice call /modify_string_service "inputstring: 'Let us learn ROS.'"
    ```

#### Rosbag record and play
- To run rosbag recording run the following in a new terminal (assuming the above nodes are running):
    ```
    cd ~/pubsub_ws/
    source devel/setup.bash
    rosbag record -O record_topics.bag -a --duration 15

    # -a: to record all topics
    # -O record_topics.bag: name of the output bag file
    # --duration 15: record for 15 sec 
    ```

- To verify the recording:
    - Examining the bag file:
        ```
        rosbag info <your bagfile>
        ```
    - Stop the /talker node and only start the listner node
        ```
        cd ~/pubsub_ws/
        source devel/setup.bash
        rosrun beginner_tutorials listener
        ```
    - Replay the bag file: 
        ```
        rosbag play record_topics.bag
        ```
    - You can verify that the listener is now subscribing to the recorded messages.

- Run a sample recorded rosbag [file](results/bag).
    - In a new terminal start `rosmaster`:
        ```
        roscore
        ```
    - In a new terminal start the listner of the `/chatter` topic:
        ```
        cd ~/pubsub_ws/
        source devel/setup.bash
        rosrun beginner_tutorials listener
        ```
    - In a new terminal replay the bag file. This is a 15 sec recording of many topics including `/chatter` topic.
        ```
        cd ~/pubsub_ws/
        source devel/setup.bash
        roscd beginner_tutorials
        cd results/bag
        ```
    - Examining the bag file: [Sample output](results/rosbag_info.png).
        ```
        rosbag info record_topics.bag
        ```
    - Play the bag file: [Sample output](results/rosbag_replay_demo.png).
        ```
        rosbag play record_topics.bag
        ```

### Running the tests
- Build the test:
    ```
    cd ~/pubsub_ws/
    catkin_make
    catkin_make run_tests (to run all the test executables)
    (or)
    catkin_make run_tests talker_test (to run specific executable)
    ```
- tests can be run using `rostest` too, once `$catkin_make run_tests` have been run once. 
    ```
    source devel/setup.bash
    rostest beginner_tutorials publisher_test.test
    ```


