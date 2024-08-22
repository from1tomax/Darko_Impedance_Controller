# whole_body_impedance_controller 

## change panda_gazebo controller
Find the launch file  `~/darko_ws/gazebo_panda/launch/panda_sim_controllers.launch`, change the [defaulte "ON" controller](https://github.com/justagist/gazebo_panda/blob/81bc92f09519562fcfcab5b2911aabf87669e097/launch/panda_sim_controllers.launch#L12-L17) to `gazebo_panda/effort_joint_torque_controller`, and comment the [defaulte "OFF" controller](https://github.com/justagist/gazebo_panda/blob/81bc92f09519562fcfcab5b2911aabf87669e097/launch/panda_sim_controllers.launch#L18-L31).

## build and start Gazebo environment
After you successfully enter the docker environment
```
cd ～/darko_ws/darko_repos
git clone git@github.com:from1tomax/Darko_Impedance_Controller.git
cd ..
catkin build whole_body_impedance_controller
roslaunch darko_gazebo_sim single_robot_non_native.launch
```

## launch controller
- `roslaunch arm_impedance_controller arm_impedance_controller.launch` to see the arm follow some trajectory with compliance behavior
- `roslaunch base_impedance_controller base_impedance_controller.launch` to see the base move forward
- `rosrun whole_body_impedance_controller whole_body_impedance_controller_node` to see end effector trys to follow a line along X-axis, however still not stable

