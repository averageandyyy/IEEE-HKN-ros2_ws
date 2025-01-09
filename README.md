# Pre-requisites
Make sure to be running this on an Ubuntu-based OS (WSL2 if Windows) and to have ROS2 (`ros-humble` was used here) installed.

# Contents
This repository contains 3 packages. `simple_publisher` for a basic publisher, `simple_subscriber` for a basic subscriber and `turtlefun` that interacts with `turtlesim` to move a virtual turtle to a desired `Pose`.

# Building
Once you have cloned the repository, just `cd` into it and run
```
colcon build
source install/setup.bash
```

# Running the basic publisher and subscriber
You will need at least two terminals for this. Be sure to have `source`d the `install/setup.bash` before doing this.
On one terminal, run
```
ros2 run simple_publisher yapper
```
On the other, run
```
ros2 run simple_subscriber listener
```
And observe the output on both terminals! This is a basic publisher and subscriber setup to give participants an intuition behind ROS concepts like Nodes, Topics, Publishers and Subscribers.

# Running `turtlefun`
You will need at least 5 terminals for the full experience
On one terminal, start up `turtlesim` with
```
ros2 run turtlesim turtlesim_node
```
You should see a GUI application with a virtual turtle in the center pop up.

On another terminal, run
```
ros2 topic echo /turtle1/pose
```
to see where the turtle is currently positioned and facing.

On another, run
```
ros2 topic echo /turtle1/cmd_vel
```
to see the kinematics (velocities) of the turtle.

On another, run
```
ros2 run turtlefun turtle
```
This node listens to where the turtle currently is and where it is supposed to go and moves towards its destination by first moving towards the desired position before adopting the desired orientation. Without any changes, the turtle is currently set to move approximately towards the top left of the GUI and face the left.

Lastly, run
```
ros2 run turtlefun destination_publisher
```
to publish a dummy/test destination for the turtle to move towards

This example is a healthy step up from the previous basic subscriber-publisher example demonstrating how ROS2 can/is be used in more complex robotics operations involving movement and navigation and how regular Python programming, software engineering and ROS2 can culminate in complex robotic operations.
