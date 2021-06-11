params is in encoder_publisher_ws/src/local_planner/params/icr_params.yaml
launch file is in local_planner/launch/icr_launch.launch
codes used:
1. local_planner/src/icr_search.cpp
2. local_planner/include/local_planner/icr_utils.h

to run:

1st terminal:
-------------
rostopic pub -r 10 /can_encoder geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.7
  z: 0.7"

2nd terminal:
-------------
roslaunch local_planner icr_launch.launch

3rd terminal:
-------------
run any rosbag

4th terminal:
-------------
rostopic pub /turn_angle std_msgs/Float64 "data: x.xx"
- data to publish is in radians
