# yahboom_rosmaster 

This is an attempt to build and visualize a [Yahboom](https://github.com/YahboomTechnology/ROSMASTERX3) mobile robot with URDF in ROS2 Humble following [Automatic Addison's Tutorial](https://automaticaddison.com/create-and-visualize-a-mobile-robot-with-urdf-ros-2-jazzy/).


## What I learnt first:

1. Mesh files - makes your robot look realistic in simulation. Typically in formats such as STL. One which is used in 3D printing and designed using CAD programs.

2. URDF files are created in XACRO formats which are like blueprints. If XACRO is the architect drawing up plans, URDF is the ready-to-use construction document. XACRO offers more flexibility and organization. But XACRO must be translated into compatible URDF format for ROS.

3. How modular components are defined, parameterized,  and then clubbed together into one robot model.

4. Needed to add this `sudo apt-get install ros-${ROS_DISTRO}-urdf-tutorial` and could visualize the model with `ros2 launch urdf_tutorial display.launch.py model:=/~{workspace}~/src/yahboom_rosmaster/yahboom_rosmaster_description/urdf/robots/rosmaster_x3.urdf.xacro`


<div align=center>
<img src="./yahboom_rosmaster_description/robot_img.png" alt="Yahboom RosMaster Mobile Robot" width="480">
</div>

## Next step is running simulation in Gazebo:

### Define control packages and mecanum wheel controller

1. First I added ros2 control packages and required dependencies, but the example didn't work following the tutorial instruction. Issue is that the topic is actually different (unstamped version). So the correct way to execute is -
`ros2 launch gz_ros2_control_demos diff_drive_example.launch.py`

and

`ros2 topic pub /diff_drive_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --rate 5`

2. Second I created an empty world. The way to launch with ros2 humble compatible ignition is:
`ign sim path_to_world`
Added model files for the worlds.

Also, there might be potential problems with using gz-gazebo... plugins. Need to keep in mind.

3. Proceeded to create a mecanum drive controller. Defined header and source files. There was a problem because of ROS2 Humble API compatibility issues since the actual code was written for a later version. Following are the changes which were needed.

a. RealtimeBox API Changes - error: cannot convert 'lambda' to 'const std::shared_ptr<geometry_msgs::msg::TwistStamped_<std::allocator<void> > >&'

While newer ROS2 version allows lambda function, humble expects direct value assignment:
`received_velocity_msg_ptr_.set(msg);`

b. Hardware Interface return type changes - error: could not convert 'void' to 'bool'
612 |       if (!wheel_handle.velocity->get().set_value(0.0)) 

The older humble version returns void and doesn't check for errors while newer jazzy one returns bool indicating success/failure:
`wheel_handle.velocity->get().set_value(0.0);`

c. State Interface API - 

// New Jazzy: get_optional() for safe access
wheel_handles_[FRONT_LEFT].feedback->get().get_optional().value_or(0.0)

// Old (Humble): Direct get_value() access
`wheel_handles_[FRONT_LEFT].feedback->get().get_value()`

d. Lifecycle State Access -

Needed to fetch state using node accessor in Humble version:
`get_node()->get_current_state().id()`

### conneting Gazebo to ROS2 control

1. Defined controller manager which can manage multiple controllers. Along with it added multiple launch and config files as needed.

2. To launch Gazebo (alias x3 in tutorial):
`bash ~/projects/project_ws/src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_gazebo.sh`

or simply:
`bash src/yahboom_rosmaster/yahboom_rosmaster_bringup/scripts/rosmaster_x3_gazebo.sh` from inside workspace

After waiting for sometime, the environment and everything does pop up. But I think there's still a lot of configuration issues because of ros2 humble incompatiblity. Need to take a look at it later. For e.g.
[ruby $(which ign) gazebo-3] [Err] [SystemPaths.cc:482] File [/home/ubuntu/ros2_ws/install/yahboom_rosmaster_gazebo/share/yahboom_rosmaster_gazebo/models/cafe_table/materials/textures/Maple.jpg] resolved to path [/home/ubuntu/ros2_ws/install/yahboom_rosmaster_gazebo/share/yahboom_rosmaster_gazebo/models/cafe_table/materials/textures/Maple.jpg] but the path does not exist
[ruby $(which ign) gazebo-3] [Err] [Material.cc:164] Unable to find texture [/home/ubuntu/ros2_ws/install/yahboom_rosmaster_gazebo/share/yahboom_rosmaster_gazebo/models/cafe_table/materials/textures/Maple.jpg] as a locally cached texture or in path [/home/amlansahoo07/projects/project_ws/install/yahboom_rosmaster_gazebo/share/yahboom_rosmaster_gazebo/models/cafe_table/meshes]

3. To move the robot -
`ros2 topic pub /mecanum_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{header: {stamp: {sec: $(date +%s), nanosec: 0}, frame_id: ''}, twist: {linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"`

4. To check the active controllers -
`ros2 control list_controllers` and `ros2 control list_controllers -v`

5. Finally implemented a script to run the robot usin:
`ros2 run yahboom_rosmaster_system_tests square_mecanum_controller`

![alt text](image.png)

<div align=center>
<img src="./yahboom_rosmaster_system_tests/robot_script.png" alt="Robot Square Path" width="480">
</div>