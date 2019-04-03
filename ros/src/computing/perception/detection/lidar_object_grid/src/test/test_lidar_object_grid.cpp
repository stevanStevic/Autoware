#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "gtest/gtest.h"
#include "lidar_object_grid/lidar_object_grid.hpp"

namespace LidarDetector {
namespace OcuppancyGrid3D {

class LidarObjectGridTest : public testing::Test, public LidarObjectGrid
{
protected:
private:
  ros::NodeHandle nodeHandle; //!< ROS node handle
};

class FilterTest : public LidarObjectGridTest
{
protected:
  virtual void SetUp()
  {
    pcl::PointXYZ point0(-5, -5, 1);
    pcl::PointXYZ point1(0, 0, 1);
    pcl::PointXYZ point2(1, 1, 1);
    pcl::PointXYZ point3(2, 2, 1);
    pcl::PointXYZ point4(5, 5, 1);

    cloud.push_back(point0);
    cloud.push_back(point1);
    cloud.push_back(point2);
    cloud.push_back(point3);
    cloud.push_back(point4);

    initNode();
  }

  bool rv;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> filteredCloud;
};

class ProcessingTest : public LidarObjectGridTest
{
protected:
  virtual void SetUp()
  {
    pcl::PointXYZ point0(1.0, 1.0, 1.0);
    pcl::PointXYZ point1(1.1, 1.1, 2.0);
    pcl::PointXYZ point2(1.2, 1.2, 3.0);
    pcl::PointXYZ point3(1.3, 1.3, 4.0);
    pcl::PointXYZ point4(1.4, 1.4, 5.0);

    cloud.push_back(point0);
    cloud.push_back(point1);
    cloud.push_back(point2);
    cloud.push_back(point3);
    cloud.push_back(point4);

    pcl::PointXYZ point5(0.0, 0.0, 1.0);
    pcl::PointXYZ point6(1.0, 1.0, 1.0);
    pcl::PointXYZ point7(2.0, 2.0, 1.0);
    pcl::PointXYZ point8(3.0, 3.0, 1.0);

    cloudSpread.push_back(point5);
    cloudSpread.push_back(point6);
    cloudSpread.push_back(point7);
    cloudSpread.push_back(point8);
  }

  CellGrid grid;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> cloudSpread;
};

class GenerateSingleMarker : public LidarObjectGridTest
{
};

class GenerateMultipleMarkers : public LidarObjectGridTest
{
protected:
  virtual void SetUp()
  {
    int gridX = 12;
    int gridY = 8;

    grid.resize(gridX);
    for (auto i = 0; i < gridX; ++i)
    {
      grid[i].resize(gridY);
      for (auto j = 0; j < gridY; ++j)
      {
        grid[i][j].m_pointCount = 5;
        grid[i][j].m_height = 3;
      }
    }
  }

  CellGrid grid;
  int cellSize = 1;
};

TEST_F(GenerateMultipleMarkers, testAlphaCalculation)
{
  float alpha = calculateCellAlpha(150);

  EXPECT_EQ(0.5f, alpha);
}

TEST_F(GenerateMultipleMarkers, multiMarkerGenerationNonEmptyGrid)
{
  std::vector<visualization_msgs::Marker> markers;
  bool rv = generateMarkers(grid, cellSize, markers);

  EXPECT_TRUE(rv);
  EXPECT_FALSE(markers.empty());
}

TEST_F(GenerateMultipleMarkers, multiMarkerGenerationEmptyGrid)
{
  CellGrid emptyGrid;
  std::vector<visualization_msgs::Marker> markers;
  bool rv = generateMarkers(emptyGrid, cellSize, markers);

  EXPECT_FALSE(rv);
}

TEST_F(GenerateMultipleMarkers, multiMarkerGenerationCheckSize)
{
  std::vector<visualization_msgs::Marker> markers;
  bool rv = generateMarkers(grid, cellSize, markers);

  EXPECT_TRUE(rv);
  EXPECT_EQ(grid.size() * grid[0].size(), markers.size());
}

TEST_F(GenerateMultipleMarkers, multiMarkerGenerationCheckMarker)
{
  std::vector<visualization_msgs::Marker> markers;
  CellGrid gridBackup = grid;

  bool rv = generateMarkers(grid, 1, markers);

  EXPECT_TRUE(rv);
  EXPECT_EQ(gridBackup[0][0].m_height, markers[0].scale.z);
}

TEST_F(GenerateMultipleMarkers, multiMarkerCheckGridCleanup)
{
  std::vector<visualization_msgs::Marker> markers;

  bool rv = generateMarkers(grid, cellSize, markers);

  EXPECT_TRUE(rv);

  for(auto x = 0; x < grid.size(); ++x)
  {
    for(auto y = 0; y < grid[x].size(); ++y)
    {
      EXPECT_EQ(grid[x][y].m_pointCount, 0);
      EXPECT_EQ(grid[x][y].m_height, -1.f);
    }
  }
}

TEST_F(GenerateSingleMarker, markerGeneration)
{
  int posX = 1;
  int posY = 1;
  int posZ = 1;
  float alpha = 1.f;
  int scale = 1;
  float height = 1.f;

  visualization_msgs::Marker marker =
      generateMarker(posX, posY, posZ,
                     alpha, scale, height);

  EXPECT_EQ(posX, marker.pose.position.x);
  EXPECT_EQ(posY, marker.pose.position.y);
  EXPECT_EQ(posZ, marker.pose.position.z);
  EXPECT_EQ(alpha, marker.color.a);
  EXPECT_EQ(scale, marker.scale.x);
  EXPECT_EQ(height, marker.scale.y);
}

TEST_F(ProcessingTest, passValidSize)
{
  int xSize = 12;
  int ySize = 8;

  bool rv = processCloud(cloud, xSize, ySize, 1, grid);
  EXPECT_TRUE(rv);

  EXPECT_EQ(grid.size(), xSize);
  for (auto i = 0; i < grid.size(); ++i)
  {
    EXPECT_EQ(grid[i].size(), ySize);
  }
}

TEST_F(ProcessingTest, passInvalidSize)
{
  bool rv = processCloud(cloud, -5, -5, 1, grid);
  EXPECT_FALSE(rv);
}

TEST_F(ProcessingTest, checkCounting)
{
  int xSize = 12;
  int ySize = 8;
  int scale = 1;

  bool rv = processCloud(cloud, xSize, ySize, scale, grid);
  EXPECT_TRUE(rv);

  // All points should fall into same cell in the grid
  EXPECT_EQ(grid[7][5].m_pointCount, 5.f);
}

TEST_F(ProcessingTest, checkCountingSpread)
{
  int xSize = 12;
  int ySize = 8;
  int scale = 1;

  bool rv = processCloud(cloudSpread, xSize, ySize, scale, grid);

  EXPECT_EQ(grid[6][4].m_pointCount, 1.f);
  EXPECT_EQ(grid[7][5].m_pointCount, 1.f);
  EXPECT_EQ(grid[8][6].m_pointCount, 1.f);
  EXPECT_EQ(grid[9][7].m_pointCount, 1.f);
}

TEST_F(ProcessingTest, checkHeight)
{
  int xSize = 12;
  int ySize = 8;
  int scale = 1;

  bool rv = processCloud(cloud, xSize, ySize, scale, grid);

  // Expected height is 5.0 which comes from point4 (max z in that cell)
  EXPECT_EQ(grid[7][5].m_height, 5.f);
}

TEST_F(FilterTest, passFilter)
{
  filterROI(cloud.makeShared(), filteredCloud, 6, 6, 3, 1, 0.1f);

  EXPECT_TRUE(rv);
}

TEST_F(FilterTest, correctlyFiltered)
{
  filterROI(cloud.makeShared(), filteredCloud, 6, 6, 3, 1, 0.1f);

  EXPECT_EQ(3, filteredCloud.size());
}


} // namespace OcuppancyGrid3D
} // namespace LidarDetector

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_object_grid_test");
  ::testing::InitGoogleTest(&argc, argv);

  ros::shutdown();
  return RUN_ALL_TESTS();
}
