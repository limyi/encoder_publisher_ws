automation.launch:
------------------
roslaunch local_planner automation.launch record_pose:=true/false
# params in local_planner/params/cfg.yaml

icr_launch.launch:
------------------
roslaunch local_planner icr_launch
# params in local_planner/params/icr_params.yaml

sonar.launch:
-------------
roslaunch local_planner sonar.launch
