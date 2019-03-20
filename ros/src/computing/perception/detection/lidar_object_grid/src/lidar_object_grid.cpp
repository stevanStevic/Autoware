#include "lidar_object_grid/lidar_object_grid.hpp"

LidarObjectGrid::LidarObjectGrid()
{

}

bool LidarObjectGrid::getTransforamtion(tf::StampedTransform& transformation)
{
  try
  {
    m_transformListener.waitForTransform("base_link",
                                         "velodyne",
                                         ros::Time(0),
                                         ros::Duration(1.0));

    m_transformListener.lookupTransform("base_link",
                                        "velodyne",
                                        ros::Time(0),
                                        transformation);
  }
  catch(tf::TransformException& e)
  {
    ROS_INFO("%s", e.what());
    return false;
  }

  return true;
}
