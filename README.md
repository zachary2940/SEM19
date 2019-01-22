# NV11_catvehicle
This repository contains simulators for catvehicle and RBCar

#### Steps to run RBCar
1. Run the gazebo simulation - `roslaunch rbcar_sim_bringup rbcar_complete_gs.launch`
2. Launch the gmapping node - `roslaunch rbcar_localization slam_gmapping.launch`
3. Launch the move_base node with teb planner - `roslaunch rbcar_navigation move_base.launch`
4. Launch RViz - `roslaunch rbcar_description rbcar_rviz.launch`

Note: The car is hidden behind the gas station in the RViz (haven't modified it yet)
