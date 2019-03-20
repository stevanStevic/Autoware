#ifndef __LIDAR_OBJECT_GRID__
#define __LIDAR_OBJECT_GRID__

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

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

private:
  tf::TransformListener m_transformListener; //!< Listener for acquiring transformation
};

#endif //__LIDAR_OBJECT_GRID__
