# NV11_catvehicle
This repository contains simulators for catvehicle and RBCar

#### Steps to run RBCar
1. Run the gazebo simulation - `roslaunch rbcar_sim_bringup rbcar_complete_gs.launch`
2. Launch the amcl node - `roslaunch rbcar_localization amcl.launch`
3. Launch the move_base node with teb planner - `roslaunch rbcar_nav move_base.launch`
4. Launch RViz - `roslaunch rbcar_description rbcar_rviz.launch`

#### To Activate waypoints following:
Default waypoints csv file for gas station - `roslaunch select_waypoints select_waypoints`