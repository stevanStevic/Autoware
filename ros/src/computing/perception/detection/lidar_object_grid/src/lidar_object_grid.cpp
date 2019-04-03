#include <pcl/filters/conditional_removal.h>

#include "lidar_object_grid/lidar_object_grid.hpp"

namespace LidarDetector {
namespace OcuppancyGrid3D {

LidarObjectGrid::LidarObjectGrid() : m_latestCloudMessage(nullptr),
                                     m_inputTopic(""),
                                     m_outputTopic(""),
                                     m_outputCoordianteFrame(""),
                                     m_gridXSize(0),
                                     m_gridYSize(0),
                                     m_cellSize(0),
                                     m_maxPointsPerCell(300),
                                     m_objectHeight(0)
{
}

bool LidarObjectGrid::initNode()
{
  nodeHandle.param<std::string>("inputTopic", m_inputTopic, "points_raw");
  nodeHandle.param<std::string>("outputTopic", m_outputTopic, "visualization_marker");
  nodeHandle.param<std::string>("outputCordinateFrame", m_outputCoordianteFrame, "base_link");
  nodeHandle.param("gridX", m_gridXSize, 12);
  nodeHandle.param("gridY", m_gridYSize, 8);
  nodeHandle.param("cellSize", m_cellSize, 1);
  nodeHandle.param("maxPoints", m_maxPointsPerCell, 300);
  nodeHandle.param("objectH", m_objectHeight, 5);

  ROS_INFO("Params:");
  ROS_INFO("inputTopic: %s", m_inputTopic.c_str());
  ROS_INFO("outputTopic: %s", m_outputTopic.c_str());
  ROS_INFO("outputCordinateFrame: %s", m_outputCoordianteFrame.c_str());
  ROS_INFO("GridX: %d", m_gridXSize);
  ROS_INFO("GridY: %d", m_gridYSize);
  ROS_INFO("cellSize: %d", m_cellSize);
  ROS_INFO("maxPoints: %d", m_maxPointsPerCell);
  ROS_INFO("objectH: %d", m_objectHeight);

  if ((m_gridXSize & 0x1 != 0) || (m_gridXSize <= 0) ||
      (m_gridYSize & 0x1 != 0) || (m_gridYSize <= 0) ||
      (m_objectHeight <= 0) || (m_cellSize <= 0))
  {
    return false;
  }

  int xActualSize = m_gridXSize * m_cellSize;
  int yActualSize = m_gridYSize * m_cellSize;
  int xActualHeight = m_objectHeight * m_cellSize;

  // Create filter
  m_rangeCond = boost::make_shared<pcl::ConditionAnd<pcl::PointXYZ>>();
  m_rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(
      "x", pcl::ComparisonOps::GT, -xActualSize / 2 + Treshold)));
  m_rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(
      "x", pcl::ComparisonOps::LT, xActualSize / 2 - Treshold)));
  m_rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(
      "y", pcl::ComparisonOps::GT, -yActualSize / 2 + Treshold)));
  m_rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(
      "y", pcl::ComparisonOps::LT, yActualSize / 2 - Treshold)));
  m_rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(
      "z", pcl::ComparisonOps::GT, 0.2))); // Remove ground
  m_rangeCond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>(
      "z", pcl::ComparisonOps::LT, xActualHeight)));

  // Subscribe to points raw topic
  m_pointCloudSub = nodeHandle.subscribe(m_inputTopic, 1, &LidarObjectGrid::pointCloudCallback, this);

  // Advertise markers topic
  m_markerPub = nodeHandle.advertise<visualization_msgs::Marker>(m_outputTopic, 1);

  ROS_INFO("Init node - complete");

  return true;
}

void LidarObjectGrid::startNode()
{
  ros::Rate loopRate(10);

  ROS_INFO("Starting node");

  while (ros::ok())
  {
    ros::spinOnce();

    tf::StampedTransform transforamtion;
    bool rv = getTransforamtion(transforamtion);
    if (!rv)
    {
      // Don't do any processing if
      // transforamtion is not acquired succesfully
      continue;
    }

    // If no msg is acquired don't execute loop
    // Protection for initial
    if (m_latestCloudMessage == nullptr)
    {
      continue;
    }

    // Convert received msg to point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*m_latestCloudMessage, *cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    ;
    pcl_ros::transformPointCloud(*cloud, *transformedCloud, transforamtion);

    pcl::PointCloud<pcl::PointXYZ> filteredCloud;
    filterROI(transformedCloud, filteredCloud);

    // If ROI filtering is not successful, continue
    if (!rv)
    {
      continue;
    }

    CellGrid grid;
    rv = processCloud(filteredCloud, m_gridXSize,
                      m_gridYSize,
                      m_cellSize,
                      grid);
    if (!rv)
    {
      continue;
    }

    std::vector<visualization_msgs::Marker> markers;
    rv = generateMarkers(grid, m_cellSize, markers);
    if (!rv)
    {
      continue;
    }

    // Publish each marker
    for (auto &m : markers)
    {
      m_markerPub.publish(m);
    }
  }
}

bool LidarObjectGrid::getTransforamtion(tf::StampedTransform &transformation)
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
  catch (tf::TransformException &e)
  {
    ROS_INFO("%s", e.what());
    return false;
  }

  return true;
}

void LidarObjectGrid::filterROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pointCloud,
                                pcl::PointCloud<pcl::PointXYZ> &filteredCloud)
{
  pcl::ConditionalRemoval<pcl::PointXYZ> conditionExecutor;

  conditionExecutor.setCondition(m_rangeCond);
  conditionExecutor.setInputCloud(pointCloud);
//  conditionExecutor.setKeepOrganized(true);

  // Apply filter
  conditionExecutor.filter(filteredCloud);

  // Remove nans
//  std::vector<int> indices;
//  pcl::removeNaNFromPointCloud(filteredCloud, filteredCloud, indices);
}

bool LidarObjectGrid::processCloud(const pcl::PointCloud<pcl::PointXYZ> &filteredCloud,
                                   const int xGridSize,
                                   const int yGridSize,
                                   const int scale,
                                   CellGrid &grid)
{
  // Check for negative size
  if (xGridSize < 0 || yGridSize < 0)
  {
    return false;
  }

  // Resize the grid to appropriate size
  grid.resize(xGridSize);
  for (auto i = 0; i < grid.size(); ++i)
  {
    grid[i].resize(yGridSize);
  }

  for (auto i = 0; i < filteredCloud.size(); ++i)
  {
    float xx = filteredCloud.points[i].x + xGridSize / 2;
    float yy = filteredCloud.points[i].y + yGridSize / 2;

    int xInd = (int)floor(xx / scale);
    int yInd = (int)floor(yy / scale);

    grid[xInd][yInd].m_pointCount++;

    if (filteredCloud.points[i].z > grid[xInd][yInd].m_height)
    {
      grid[xInd][yInd].m_height = filteredCloud.points[i].z;
    }
  }

  return true;
}

visualization_msgs::Marker LidarObjectGrid::generateMarker(const float posX, const float posY,
                                                           const float posZ,
                                                           const float alpha,
                                                           const int scale,
                                                           const float height)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = m_outputCoordianteFrame;
  marker.header.stamp = ros::Time();
  marker.ns = std::to_string(static_cast<int>(posX)) + std::to_string(static_cast<int>(posY));
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = static_cast<float>(posX);
  marker.pose.position.y = static_cast<float>(posY);
  marker.pose.position.z = static_cast<float>(posZ);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0; // Straight up orientetaion
  marker.scale.x = static_cast<float>(scale);
  marker.scale.y = static_cast<float>(scale);
  marker.scale.z = height;
  marker.color.a = alpha;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  return marker;
}

bool LidarObjectGrid::generateMarkers(CellGrid &grid, const int scale,
                                      std::vector<visualization_msgs::Marker> &markers)
{
  // Test vehicle isolation points
  int carY1 = -2;
  int carY2 = 2;
  int carX1 = -2;
  int carX2 = 4;

  if (grid.empty())
  {
    return false;
  }

  int xGridSize = grid.size();
  int yGridSize = grid[0].size();

  for (auto x = 0; x < grid.size(); ++x)
  {
    for (auto y = 0; y < grid[0].size(); ++y)
    {
      int xDraw = (x - xGridSize / 2) + scale / 2;
      int yDraw = (y - yGridSize / 2) + scale / 2;

      // Don't consider detections from test vehicle itself
      int count = 0;
      if ((xDraw > carX1 && xDraw < carX2) &&
          (yDraw > carY1 && yDraw < carY2))
      {
        count = 0;
      }
      else
      {
        count = grid[x][y].m_pointCount;
      }

      markers.push_back(
          generateMarker(
              xDraw,
              yDraw,
              grid[x][y].m_height / 2,
              calculateCellAlpha(count),
              scale,
              grid[x][y].m_height));

      grid[x][y].m_pointCount = 0;
      grid[x][y].m_height = Cell::InitialHeight;
    }
  }

  return true;
}

void LidarObjectGrid::pointCloudCallback(const sensor_msgs::PointCloud2Ptr &cloudMsg)
{
  // Acquire latest point cloud message pointer
  m_latestCloudMessage = cloudMsg;
}

} // namespace OcuppancyGrid3D
} // namespace LidarDetector
