#include "lidar_object_grid/lidar_object_grid.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_object_grid");

    LidarDetector::OcuppancyGrid3D::LidarObjectGrid node;

    if(node.initNode() == false)
    {
        ROS_ERROR("Init failed\n");
        exit(EXIT_FAILURE);
    }

    node.startNode();

    ros::shutdown();

    return 0;
}
