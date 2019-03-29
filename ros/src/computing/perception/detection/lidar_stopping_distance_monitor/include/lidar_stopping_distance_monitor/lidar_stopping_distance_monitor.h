#ifndef __LIDAR_STOPPING_DISTANCE_MONITOR__
#define __LIDAR_STOPPING_DISTANCE_MONITOR__

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <sensor_msgs/PointCloud2.h>

class LidarStoppingDistanceMonitor
{
public:
  static constexpr float Treshold = 0.1f;

  /*
     * Ctor
     */
  LidarStoppingDistanceMonitor();

protected:
  bool filterROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                 pcl::PointCloud<pcl::PointXYZ> &outputCloud,
                 const double length,
                 const double width,
                 const double offsetX,
                 const double maxH,
                 const double minH);

  double calculateBreakingDistance(double currentVel, double prevVel, double maxDeceleration)
  {
    double averageVel = (prevVel + currentVel) / 2;
    return (-averageVel * averageVel) / (2 * maxDeceleration);
  }

private:
  ros::NodeHandle m_nodeHandle; //!< Node handle

  //Node Params
  std::string m_outputCoordinateFrame; //!< Coordinate frame
  double m_laneWidth;                  //!< Lane width
  double m_roiLen;                     //!< ROI length
  double m_maxH;                       //!< Maximum object height in ROI
  double m_minH;                       //!< Minimum object height in ROI
  double m_maxVehicleDeceleration;     //!< Vehicle maximum deceleration
};

#endif // __LIDAR_STOPPING_DISTANCE_MONITOR__