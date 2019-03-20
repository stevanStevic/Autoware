#include "gtest/gtest.h"
#include "lidar_object_grid/lidar_object_grid.hpp"

class LidarObjectGridTest : public testing::Test, public LidarObjectGrid
{
protected:

private:
  ros::NodeHandle nodeHandle; //!< ROS node handle
};

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
