# Kangaroo_x2_driver for ROS2 environment.
*A driver for the motion controller Kangaroo X2, used with the motor controller Sabertooth dual.
It accepts measurements in radians / second for the two wheels on the JointTrajectory topic, and publishes odometry to the JointState topic.*

Rewritten in ROS2 API. Based on original ROS1 smd-ros-devel [package](https://github.com/smd-ros-devel/kangaroo_x2_driver)

### Installation
Create new workspace and 'src' folder. 

Clone the package inside. 

Build package. 
```
mkdir -p /kangaroo_ws/src
cd kangaroo_ws/src/
git clone https://github.com/then-ck/kangaroo_x2_driver_ROS2.git
cd ..
colcon build
```

### Note
Colcon build will give 'stderr', don't panic, program still works.   

### Execute program
```
ros2 run kangaroo_ros2 kangaroo_driver_node
```
