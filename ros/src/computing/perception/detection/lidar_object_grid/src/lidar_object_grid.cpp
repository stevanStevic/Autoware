#include <pcl/filters/conditional_removal.h>

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

bool LidarObjectGrid::filterROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloud,
                                pcl::PointCloud<pcl::PointXYZ>& filteredCloud,
                                const int xGridSize,
                                const int yGridSize,
                                const int height,
                                const int scale,
                                const float thrashold)
{
  if((xGridSize & 0x1 != 0) || (xGridSize <= 0) ||
     (yGridSize & 0x1 != 0) || (yGridSize <= 0) ||
     (height <= 0) || (scale <= 0) || thrashold <= 0)
  {
    return false;
  }

  int xActualSize = xGridSize * scale;
  int yActualSize = yGridSize * scale;
  int xActualHeight = height * scale;

  // Set filters
  pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
                                                    pcl::ConditionAnd<pcl::PointXYZ> ());
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (
                                                                              "x", pcl::ComparisonOps::GT, -xActualSize / 2 + thrashold)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (
                                                                              "x", pcl::ComparisonOps::LT, xActualSize / 2 - thrashold)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (
                                                                              "y", pcl::ComparisonOps::GT, -yActualSize / 2 + thrashold)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (
                                                                              "y", pcl::ComparisonOps::LT, yActualSize / 2 - thrashold)));
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (
                                                                              "z", pcl::ComparisonOps::GT, 0.2)));  // Remove ground
  range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ> (
                                                                              "z", pcl::ComparisonOps::LT, xActualHeight)));
  pcl::ConditionalRemoval<pcl::PointXYZ> conditionExecutor;
  conditionExecutor.setCondition(range_cond);
  conditionExecutor.setInputCloud(pointCloud);
  conditionExecutor.setKeepOrganized(true);

  // Apply filter
  conditionExecutor.filter(filteredCloud);

  // Remove nans
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(filteredCloud, filteredCloud, indices);

  return true;
}

bool LidarObjectGrid::processCloud(const pcl::PointCloud<pcl::PointXYZ>& filteredCloud,
                                           const int xGridSize, 
                                           const int yGridSize,
                                           const int scale,
                                           CellGrid& grid)
{
  // Check for negative size
  if(xGridSize < 0 || yGridSize < 0)
  {
    return false;
  }

  // Resize the grid to appropriate size
  grid.resize(xGridSize);
  for(auto i = 0; i < grid.size(); ++i)
  {
    grid.at(i).resize(yGridSize);
  }

  for(auto i = 0; i < filteredCloud.size(); ++i)
  {
    float xx = filteredCloud.points[i].x + xGridSize / 2;
    float yy = filteredCloud.points[i].y + yGridSize / 2;

    int xInd = (int)floor(xx / scale);
    int yInd = (int)floor(yy / scale);

    grid.at(xInd).at(yInd).m_pointCount++;

    if(filteredCloud.points[i].z > grid.at(xInd).at(yInd).m_height)
    {
      grid.at(xInd).at(yInd).m_height = filteredCloud.points[i].z;
    }
  }

  return true;
}
