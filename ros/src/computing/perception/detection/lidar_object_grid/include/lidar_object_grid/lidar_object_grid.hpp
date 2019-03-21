#ifndef __LIDAR_OBJECT_GRID__
#define __LIDAR_OBJECT_GRID__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class LidarObjectGrid
{
public:
  LidarObjectGrid();

protected:
  /*!
   * \brief getTransforamtion Acquire valid transforamtion vec
   * \param transformation
   * \return true if successful
   */
  bool getTransforamtion(tf::StampedTransform& transformation);

  /*!
   * \brief filterROI filtering point cloud to leave only ROI
   * \details Grid size by X and Y must be even numbers
   * \param pointCloud    cloud to be filtered
   * \param filteredCloud filtered cloud
   * \param xGridSize     size of grid by X
   * \param yGridSize     size of grid by Y
   * \param height        height of the grid
   * \param scale         cell size
   * \param thrashold     small value not to take edge cases
   * \return true if values are correct
   */
  bool filterROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pointCloud,
                                           pcl::PointCloud<pcl::PointXYZ>& filteredCloud,
                                           const int xGridSize,
                                           const int yGridSize,
                                           const int height,
                                           const int scale,
                                           const float thrashold);

private:
  tf::TransformListener m_transformListener; //!< Listener for acquiring transformation
};

#endif //__LIDAR_OBJECT_GRID__
