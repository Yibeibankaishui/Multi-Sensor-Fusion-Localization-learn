// TODO
#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include "imu_integration/evaluator/activity.hpp"

int main(int argc, char** argv) {
    std::string node_name{"imu_intergration_evaluator_node"};
    ros::init(argc, argv, node_name);

    imu_intergration::evaluator::Activity activity;

    ros::Rate loop_rate(100);

    while (ros::ok()) {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}

