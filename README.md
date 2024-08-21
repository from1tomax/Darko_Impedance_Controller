# whole_body_impedance_controller 

After you successfully enter the docker environment
```
cd ～/darko_ws/darko_repos
git clone git@github.com:from1tomax/Darko_Impedance_Controller.git
cd ..
catkin build whole_body_impedance_controller
roslaunch darko_gazebo_sim single_robot_non_native.launch
```
- `roslaunch arm_impedance_controller arm_impedance_controller.launch` to see the arm follow some trajectory with compliance behavior
- `roslaunch base_impedance_controller base_impedance_controller.launch` to see the base move forward
- `rosrun whole_body_impedance_controller whole_body_impedance_controller_node` to see end effector trys to follow a line along X-axis, however still  not stable.
