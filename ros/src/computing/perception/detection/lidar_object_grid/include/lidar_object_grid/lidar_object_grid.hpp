#ifndef __LIDAR_OBJECT_GRID__
#define __LIDAR_OBJECT_GRID__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <vector>

constexpr float InitialHeight = -1.f;

struct Cell
{
  float m_height;
  int m_pointCount;

  Cell() : 
    m_height(InitialHeight),
    m_pointCount(0)
  {
  }
};

using Cell = struct Cell;
using CellGrid = std::vector<std::vector<Cell>>;

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

  /*!
   * \brief processCloud Create cell grid from filtered cloud
   * \param filteredCloud Preprocessed cloud with ROI isolated
   * \param xGridSize size of grid by X
   * \param yGridSize size of grid by Y
   * \param scale size of single cell
   * \param grid Output value, will be filled with appropriate cell grid
   * \return true if successful
   */
  bool processCloud(const pcl::PointCloud<pcl::PointXYZ>& filteredCloud,
                                           const int xGridSize, 
                                           const int yGridSize,
                                           const int scale,
                                           CellGrid& grid);

private:
  tf::TransformListener m_transformListener; //!< Listener for acquiring transformation
};

#endif //__LIDAR_OBJECT_GRID__
