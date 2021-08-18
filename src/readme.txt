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
	
