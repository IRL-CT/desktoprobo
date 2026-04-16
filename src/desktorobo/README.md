# mobilehri2023
Code base for mobile hri class 2023. 
Starter code for students to control hoverboard robot (differential drive controller).

- Updated [YD-LiDAR ROS 2](https://github.com/YDLIDAR/ydlidar_ros2_driver) to support ROS2 humble.
- Implemented a Pi Camera publisher through OpenCV.
- `joy_teleop_keymapping` maps joystick controller buttons to twist
- `mobile_robot_control` manages the communication with ODrive. [reference](https://github.com/neomanic/odrive_ros) for odometry.
