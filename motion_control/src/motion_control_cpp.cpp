#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include "competition.h"
#include "gantry.h"
#include "agv.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_cpp_node");
    ros::NodeHandle node;

    // 0 means that the spinner will use as many threads as there are processors on your machine.
    //If you use 3 for example, only 3 threads will be used.
    ros::AsyncSpinner spinner(0);
    spinner.start();

    // start the competition
    Competition competition(node);
    competition.init();
    std::string competition_state = competition.getCompetitionState();
    auto competition_start_time = competition.getClock();

    // initialize the gantry
    Gantry gantry(node);
    gantry.init();

    bool demo = false;

    if (demo) {
        // hardcoded part information in the world frame
        // TODO: read /ariac/orders
        // TODO: Find this part using cameras and transform its pose in the world frame
        tf2::Quaternion part_found_quaternion;
        part part_found;
        part_found.type = "assembly_pump_red";
        part_found.pose.position.x = -1.998993;
        part_found.pose.position.y = 3.279920;
        part_found.pose.position.z = 0.779616;
        // create this quaternion from roll/pitch/yaw (in radians)
        part_found_quaternion.setRPY(0, 0, 0);
        // ROS_INFO_STREAM(part_found_quaternion);
        part_found.pose.orientation.x = part_found_quaternion.getX();
        part_found.pose.orientation.y = part_found_quaternion.getY();
        part_found.pose.orientation.z = part_found_quaternion.getZ();
        part_found.pose.orientation.w = part_found_quaternion.getW();


        gantry.goToPresetLocation(gantry.home_);
        gantry.goToPresetLocation(gantry.at_bin1_);
        if (gantry.pickPart(part_found)) {
            gantry.goToPresetLocation(gantry.after_bin1_);
        }

        // hardcoded part information in the world frame
        // TODO: you should retrieve this information from /ariac/orders
        tf2::Quaternion part_in_agv_quaternion;
        part part_in_agv;
        part_in_agv.type = "assembly_pump_red";
        part_in_agv.pose.position.x = -2.015680;
        part_in_agv.pose.position.y = 4.575389;
        part_in_agv.pose.position.z = 0.809011;
        // create this quaternion from roll/pitch/yaw (in radians)
        part_in_agv_quaternion.setRPY(0, 0, -1.570796);
        // ROS_INFO_STREAM(part_found_quaternion);
        part_in_agv.pose.orientation.x = part_in_agv_quaternion.getX();
        part_in_agv.pose.orientation.y = part_in_agv_quaternion.getY();
        part_in_agv.pose.orientation.z = part_in_agv_quaternion.getZ();
        part_in_agv.pose.orientation.w = part_in_agv_quaternion.getW();


        // TODO: "agv1" is hardcoded here
        // this should be retrieved from /ariac/orders
        if (gantry.placePart(part_in_agv, "agv1")) {
            gantry.goToPresetLocation(gantry.after_agv1_);
            gantry.goToPresetLocation(gantry.after_bin1_);
            gantry.goToPresetLocation(gantry.home_);
        }
    }
    else {
        gantry.goToPresetLocation(gantry.home_);
        gantry.goToPresetLocation(gantry.at_bin1_);
        gantry.goToPresetLocation(gantry.after_bin1_);
        gantry.goToPresetLocation(gantry.before_agv1_);
        gantry.goToPresetLocation(gantry.at_agv1_);
        gantry.goToPresetLocation(gantry.after_agv1_);
        gantry.goToPresetLocation(gantry.after_bin1_);
        gantry.goToPresetLocation(gantry.home_);
    }

    // ship agv
    AGV agv_control(node);
    // TODO: The following arguments should be retrieved from /ariac/orders
    agv_control.shipAGV("order_0_kitting_shipment_0", "kit_tray_1", "as2");

    // end the competition
    competition.endCompetition();
    
    ros::waitForShutdown();
}