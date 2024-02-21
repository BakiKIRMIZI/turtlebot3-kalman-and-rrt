# Turtlebot3 Burger Kalman & RRT Project

In this project, a robot with differential drive capability, Turtlebot3 Burger, will be used in the Gazebo simulation environment. The given modified xacro file will be used for Turtlebot3 Burger.

---

The project consists of two parts:

1.  <br /> - Localization of the robot will be performed using only IMU and GPS data. A ROS node containing an (Extended) Kalman Filter will be written for this purpose. (Note: The Odom topic can only be used for comparison purposes.) <br /> - Noise levels for IMU and GPS are specified in the xacro file of the robot. Analyze the localization performance of your system for different noise levels.

2. <br /> - A ROS node will be written to implement an RRT-based motion planning algorithm that will navigate the robot to a target pose determined by the user via the /goal_pose topic. <br /> - Obstacles will be in the form of cylinders. The positions and radii of the obstacles will be read from a txt file. Each line of this txt file will contain the x and y coordinates of an obstacle along with its radius. <br /> - Test your motion planning node with different numbers and radii of obstacles.

The localization and motion planning nodes will work together.
