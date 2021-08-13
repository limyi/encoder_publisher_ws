Download Workspace:
-------------------
1. git clone https://github.com/jcch96/encoder_publisher_ws.git
2. git checkout teleop
3. git status (optional check make sure in "teleop" branch)

Set up workspace:
-----------------
1. navigate to /encoder_publisher_ws
2. delete build & devel folders
3. run: 
./ds4_install
./m

##################
## TELEOP SETUP ##
##################

Pair PC with ds4 controller using bluetooth:
1. press and hold PS and Share button until blinking white light appears
2. Go to PC bluetooth settings and pair

1st Terminal:
-------------
cd encoder_publisher_ws
./can_init.sh

2nd Terminal:
-------------
roscore

3rd Terminal:
-------------
cd encoder_publisher_ws
source devel/setup.bash
rosrun can_encoder_pub can_encoder_pub_node

4th Terminal:
-------------
cd encoder_publisher_ws
source devel/setup.bash
roslaunch panthera_locomotion run2.launch

5th Terminal:
-------------
cd encoder_publisher_ws
source devel/setup.bash
roslaunch panthera_locomotion ds4controller.launch

6th Terminal: (Optional for brushes/vacuum/actuators)
-------------
sudo chmod 777 /dev/tty*
cd encoder_publisher_ws
source devel/setup.bash
roslaunch roboclaw_node panthera.launch

## if error for roboclaw node, change exchange the numbers for ttyACM_
<arg name="dev0" default="/dev/ttyACM0"/>
<arg name="dev1" default="/dev/ttyACM1"/>
<arg name="dev2" default="/dev/ttyACM2"/>

found in encoder_publisher_ws/src/roboclaw_node/launch/panthera.launch


### WARNINGS ###
1. If 4th Terminal continuously prints "failed to read instrument", shut off power and restart everything. If error persists, could be due to motor failure.

2. After emergency stop, make sure the 4th Terminal is closed or press Ctrl + c to stop the code.

3. Make sure there are 6 values in the 3rd Terminal and no 0 values. If there are 0 values or the values are not updating, run the 1st terminal again.

###########################
## Running local planner ##
###########################
1. roslaunch local_planner automation.launch record_pose:=true
# record_pose:=true records and publishes path of robot

###############################
## Running ultrasonic sensor ##
###############################
1. roslaunch local_planner sonar.launch
# params are in local_planner/params/sonar_params.yaml

########################
## Running ICR search ##
########################
1. roslaunch local_planner icr_launch.launch
# params in local_planner/params/icr_params.yaml
# requires /occupancy_grid, /can_encoder, /turn_angle topic to be up to run search
# param to use gradient descent or brute force in param file

#########################
## Footprint publisher ##
#########################
either:
1. rosrun panthera_locomotion footprint_pub.py
# to fix footprint to velodyne

or:
1. rosrun panthera_locomotion footprint_tf.py
# publish tf between baselink and velodyne

#################
## MQTT launch ##
#################
On remote pc:
1. Connect ds4 controller to pc
2. roslaunch panthera_locomotion mqtt_pub.launch

On panthera:
1. roslaunch panthera_locomotion mqtt_sub.launch



