==================
|| ROS PACKAGES ||
==================

1. can_encoder_pub:
-------------------
	- run the siko encoders/wire encoders for steering the wheels and robot width to topic: /can_encoder
	- can run both python and cpp files to publish steering angles and robot width in twist msg:
		- linear.x = leftback wheel
		- linear.y = rightback wheel
		- linear.z = leftfront wheel
		- angular.x = rightfront wheel
		- angular.y = front wire encoder
		- angular.z = back wire encoder
	- rosrun can_encoder_pub can_encoder_pub_node

2. ds4_driver:
--------------
	- INSTALLATION:
		$ git clone https://github.com/naoki-mizuno/ds4drv --branch devel
		$ cd ds4drv
		$ python2 setup.py install --prefix ~/.local 
		##### or python3 setup.py install --prefix ~/.local
		$ sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
		$ sudo udevadm control --reload-rules
		$ sudo udevadm trigger
		$ cd ~/encoder_publisher_ws/src
		$ git clone https://github.com/naoki-mizuno/ds4_driver.git
	- run rosnode to publish ds4 controller buttons to topics: /cmd_vel, /status

3. key_command_pub (not used)
-----------------------------

4. local_planner:
-----------------
	src:
		- run zig-zag motion for panthera, ICR(instantaneous centre of rotation) search within robot base, ultrasonic sensor publisher
		- costmap_clear.cpp:
			- checks area around robot for obstacles and publishes to topic /check_cmap
		- icr_search.cpp:
			- searches for best icr for robot to rotate, sends commands to /panthera_cmd and /reconfig
		- local_planner.cpp:
			- runs zig-zag motion for robot for autonomous cleaning, subscribes to /check_cmap to avoid obstacles
			- runs based on state machine
		- pose_recorder.cpp:
			- records robot position based on pose from /ndt_matching topic run by autoware
		- sonar_costmap.cpp:
			- subscribes to ultrasonic sensor data and creates an occupancy grid layer to be fused with lidar occupancy grid layer
	launch:
		- automation.launch (zig-zag motion)
		- icr_launch.launch (icr search)
		- record_pose.launch (records pose)
		- sonar.launch (runs ultrasonic sensors)
		
5. panthera_locomotion:
-----------------------
	- runs robot locomotion code: steering motors, translational motors, connection with ds4 controller, mqtt controller, robot odometry
	src:
		- steer_motors2:
			- individual nodes for 4 steering motors
			- steer_control.py: class for steering motor
		- trans_motors;
			- individual nodes for 4 translational motors
			- trans_class.py: class for translational motors
		- footprint_pub.py: publish robot footprint fixed wrt velodyne
		- footprint_tf.py: publish robot footprint, velodyne tf wrt base_link
		- getch_modbus.py: manually rotate wheel 
		- new_controller.py: reads data from ds4_controller -> cmds for steering and traslational motors
		- path_plotter.py: publishes path taken by robot
		-	odom_bc.py &  robot_odom.py: publish robot odometry
		- motor.py: roboteq library (WIP)
		- roboteq_motor.py: roboteq motor simple rosnode
	
	launch:
		- run2.launch (runs all motors)
		- ds4_controller.launch (runs ds4 node and new_controller.py)
		- mqtt_pub.launch (runs on controller pc)
		- mqtt_sub.launch (runs on robot)
		- odom.launch (runs odometry broadcaster)
		
6. roboclaw_node:
-----------------
	- runs motors for vacuum mouth, brushes, and brush actuators
	- roslaunch roboclaw_node panthera.launch
	
7. ultrasonic_senors:
---------------------
	- runs & publish ultrasonic sensor readings
	src:
		- main.py: runs ros node
		- distance_lib.py: library to read data
		
================
|| TO INSTALL ||
================

1. velodyne:
	- http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16
	- to run: roslaunch velodyne_pointcloud VLP16_points.launch
	
2. xsens imu:
=[ Xsens MTi driver for ROS ]============================================================

Documentation:
    You can find the full documentation in "<your MT SDK directory>/doc/xsensdeviceapi/doc/html/index.html" under "ROS MTi driver" section.
Prerequisites:
    - ROS Kinetic or Melodic
    - C/C++ Compiler: GCC 5.4.0 or MSVC 14.0
    - C++11
Building:
    - Copy xsens_ros_mti_driver folder from your MT SDK directory into your catkin workspace 'src' folder.
        Make sure the permissions are set to o+rw on your files and directories
    - Build xspublic from your catkin workspace:
        $ pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
    - Build Xsens MTi driver package:
        $ catkin_make
    - Source workspace:
        $ source devel/setup.bash
Running:
    - Configure your MTi device to output desired data (e.g. for display example - orientation output)
    - Launch the Xsens MTi driver from your catkin workspace:
            $ roslaunch xsens_mti_driver xsens_mti_node.launch
        After the device has been detected, you can communicate with it from another process / terminal window.
        For example:
            $ rostopic echo /filter/quaternion
        This will result in a continuous stream of data output:
            ---
            header: 
              seq: 1386351
              stamp: 
                secs: 1545223809
                nsecs: 197252179
              frame_id: "imu_link"
            quaternion: 
              x: 0.00276306713931
              y: 0.00036825647112
              z: -0.89693570137
              w: -0.442152231932
            ---
    - There is also an example that shows a 3D visualization of the device (orientation data should be enabled in the device):
            $ roslaunch xsens_mti_driver display.launch
Notes:
    - ROS timestamps
        The data messages from devices are time stamped on arrival in the ROS driver.
        When collecting data at higher rates, eg 100 Hz, the times between reads can differ from the configured output rate in the device.
        This is caused by possible buffering in the USB/FTDI driver.
        For instance:
        10 us, 10 us, 10 us, 45 ms, 10 us, 10 us, 10 us, 39 ms, 10 us, etc.
        instead of 
        10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, 10 ms, etc.
        Work-around: for accurate and stable time stamp information, users can make use of the sensor reported time stamp (/imu/time_ref) instead.

-[ Troubleshooting ]------------------------------------------------------------
    - The Mti1 (Motion Tracker Development Board) is not recognized.
        Support for the Development Board is present in recent kernels. (Since June 12, 2015).
        If your kernel does not support the Board, you can add this manually
        $ sudo /sbin/modprobe ftdi_sio
        $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id
    - The device is recognized, but I cannot ever access the device -
        Make sure you are in the correct group (often dialout or uucp) in order to
        access the device. You can test this with
            $ ls -l /dev/ttyUSB0
            crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
            $ groups
            dialout audio video usb users plugdev
        If you aren't in the correct group, you can fix this in two ways.
        1. Add yourself to the correct group
            You can add yourself to it by using your distributions user management
            tool, or call
                $ sudo usermod -G dialout -a $USER
            Be sure to replace dialout with the actual group name if it is
            different. After adding yourself to the group, either relogin to your
            user, or call
                $ newgrp dialout
            to add the current terminal session to the group.

        2. Use udev rules
            Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules
                SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"
            Change $GROUP into your desired group (e.g. adm, plugdev, or usb).
    - The device is inaccessible for a while after plugging it in -
        When having problems with the device being busy the first 20 seconds after
        plugin, purge the modemmanager application.
    - RViz doesn't show an MTi model.
        It is a known issue with urdfdom in ROS Melodic. A workaround is to unset/modify the LC_NUMERIC environment variable:
        $ LC_NUMERIC="en_US.UTF-8"
