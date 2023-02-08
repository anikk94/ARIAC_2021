#!/bin/bash

# # spawn a part in the tray on agv1
rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.2 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_green_20
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.1 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_green_21


# trigger sensor blackout
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_green_21
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.1 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_green_22
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y -0.1 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv1::kit_tray_1::kit_tray_1::tray -model assembly_battery_green_23


# trigger high-priority order
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_green_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_green_21
# rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.20 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_battery_red_ariac/model.sdf -reference_frame agv4::kit_tray_4::kit_tray_4::tray -model assembly_battery_red_21



# # spawn a part on the tray for the assembly robot
#rosrun gazebo_ros spawn_model -sdf -x 0.0 -y 0.0 -z 0.05 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame gantry::torso_tray -model assembly_pump_blue_11

# # spawn a part in the briefcase located at assembly station 1 
rosrun gazebo_ros spawn_model -sdf -x 0.032085 -y -0.152835 -z 0.28 -R 3.14159 -P 0 -Y 0 -file `rospack find nist_gear`/models/assembly_pump_blue_ariac/model.sdf -reference_frame station1::briefcase_1::briefcase_1::briefcase -model assembly_pump_blue_12
