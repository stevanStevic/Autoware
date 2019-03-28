#include <ros/ros.h>
#include "lidar_stopping_distance_monitor/lidar_stopping_distance_monitor.h"

#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_stopping_distance_monitor");

    ros::shutdown();
    return 0;
}