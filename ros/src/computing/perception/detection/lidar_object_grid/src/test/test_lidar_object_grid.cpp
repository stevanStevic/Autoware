#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "gtest/gtest.h"
#include "lidar_object_grid/lidar_object_grid.hpp"

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
  }

  bool rv;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointCloud<pcl::PointXYZ> filteredCloud;
};

TEST_F(FilterTest, passFilter)
{
  rv = filterROI(cloud.makeShared(), filteredCloud, 6, 6, 3, 1, 0.1f);

  EXPECT_TRUE(rv);
}

TEST_F(FilterTest, lessThanZeroFilter)
{
  rv = filterROI(cloud.makeShared(), filteredCloud, -1, -1, 3, 1, 0.1f);

  EXPECT_FALSE(rv);
}

TEST_F(FilterTest, oddFilter)
{
  rv = filterROI(cloud.makeShared(), filteredCloud, 3, 3, 3, 1, 0.1f);

  EXPECT_FALSE(rv);
}

TEST_F(FilterTest, scaleFilter)
{
  int scale = 2;

  rv = filterROI(cloud.makeShared(), filteredCloud, 6, 6, 3, scale, 0.1f);

  ASSERT_TRUE(rv);

  EXPECT_EQ(5, filteredCloud.size());
}

TEST_F(FilterTest, correctlyFiltered)
{
  rv = filterROI(cloud.makeShared(), filteredCloud, 6, 6, 3, 1, 0.1f);
  std::cout << rv << std::endl;
  ASSERT_TRUE(rv);

  EXPECT_EQ(3, filteredCloud.size());
}

TEST_F(LidarObjectGridTest, aquireTransformationRetVal)
{
  tf::StampedTransform transformation;

  bool rv = getTransforamtion(transformation);

  EXPECT_TRUE(rv);
}

TEST_F(LidarObjectGridTest, aquireTransformationOutput)
{
  tf::StampedTransform correctTransformation;
  tf::StampedTransform transformation;

  // Values acquired from rostopic echo /tf
  correctTransformation.setOrigin({1.2,0.0,2.0});
  correctTransformation.setRotation({0.0, 0.0, 0.0,1.0});

  ASSERT_TRUE(getTransforamtion(transformation));

  EXPECT_EQ(correctTransformation.getOrigin(), transformation.getOrigin());
  EXPECT_EQ(correctTransformation.getRotation(), transformation.getRotation());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lidar_object_grid_test");
  ::testing::InitGoogleTest(&argc, argv);

  ros::shutdown();
  return RUN_ALL_TESTS();
}
