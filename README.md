# yahboom_rosmaster #

This is an attempt to build and visualize a [Yahboom](https://github.com/YahboomTechnology/ROSMASTERX3) mobile robot with URDF in ROS2 Humble following [Automatic Addison's Tutorial](https://automaticaddison.com/create-and-visualize-a-mobile-robot-with-urdf-ros-2-jazzy/).


What I learnt:

1. Mesh files - makes your robot look realistic in simulation. Typically in formats such as STL. One which is used in 3D printing and designed using CAD programs.

2. URDF files are created in XACRO formats which are like blueprints. If XACRO is the architect drawing up plans, URDF is the ready-to-use construction document. XACRO offers more flexibility and organization. But XACRO must be translated into compatible URDF format for ROS.

3. How modular components are defined, parameterized,  and then clubbed together into one robot model.

4. Needed to add this `sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial` and could visualize the model with `ros2 launch urdf_tutorial display.launch.py model:=/~{workspace}~/src/yahboom_rosmaster/yahboom_rosmaster_description/urdf/robots/rosmaster_x3.urdf.xacro`


<div align=center>
<img src="./yahboom_rosmaster_description/robot_img.png" alt="Yahboom RosMaster Mobile Robot" width="480">
</div>

Next step is running simualtion in Gazebo:

1. First I added ros2 control packages and required dependencies, but the example didn't work following the tutorial instruction. Issue is that the topic is actually different (unstamped version). So the correct way to execute is -
`ros2 launch gz_ros2_control_demos diff_drive_example.launch.py`

and

`ros2 topic pub /diff_drive_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --rate 5`