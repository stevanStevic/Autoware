#include "lidar_object_grid/lidar_object_grid.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_object_grid");
    LidarObjectGrid node;
    node.initNode();
    node.startNode();
    ros::shutdown();
    return 0;
}
