#include <gtest/gtest.h>
#include <ros/ros.h>

#include "lidar_stopping_distance_monitor/lidar_stopping_distance_monitor.h"

class LidarStoppingDistanceMonitorTest : public LidarStoppingDistanceMonitor, public testing::Test
{
  private:
    ros::NodeHandle nodeHandle;
};

class FilterTest : public LidarStoppingDistanceMonitorTest
{
  protected:
    virtual void SetUp()
    {
        pcl::PointXYZ point0(-5, -5, 1);
        pcl::PointXYZ point1(0, 0, 1);
        pcl::PointXYZ point2(1, 1, 1);
        pcl::PointXYZ point3(2, 2, 1);
        pcl::PointXYZ point4(5, 5, 1);
        pcl::PointXYZ point5(0.5, 0.5, 1);

        cloud.push_back(point0);
        cloud.push_back(point1);
        cloud.push_back(point2);
        cloud.push_back(point3);
        cloud.push_back(point4);
        cloud.push_back(point5);

        double offset = 5.0;

        pcl::PointXYZ point6(-5 + offset, -5, 1);
        pcl::PointXYZ point7(0 + offset, 0, 1);
        pcl::PointXYZ point8(1 + offset, 1, 1);
        pcl::PointXYZ point9(2 + offset, 2, 1);
        pcl::PointXYZ point10(5 + offset, 5, 1);
        pcl::PointXYZ point11(0.5 + offset, 0.5, 1);

        offsetCloud.push_back(point6);
        offsetCloud.push_back(point7);
        offsetCloud.push_back(point8);
        offsetCloud.push_back(point9);
        offsetCloud.push_back(point10);
        offsetCloud.push_back(point11);

        double realOffset = 9;

        pcl::PointXYZ point12(5 + realOffset, 0, 1); //T
        pcl::PointXYZ point13(10 + realOffset, 0, 1); //F
        pcl::PointXYZ point14(-9 + realOffset, 1, 1); //F
        pcl::PointXYZ point15(2 + realOffset, 2, 1); //T
        pcl::PointXYZ point16(3 + realOffset, 5, 1); //F

        appOffsetCloud.push_back(point6);
        appOffsetCloud.push_back(point7);
        appOffsetCloud.push_back(point8);
        appOffsetCloud.push_back(point9);
        appOffsetCloud.push_back(point10);
    }

    bool rv;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> offsetCloud;
    pcl::PointCloud<pcl::PointXYZ> appOffsetCloud;
    pcl::PointCloud<pcl::PointXYZ> filteredCloud;
};

TEST_F(FilterTest, filterTestInvalidInput)
{
    rv = filterROI(cloud.makeShared(), filteredCloud, -3.0, 3.0, 0.0, 1.0, -1.0);
    EXPECT_FALSE(rv);
}

TEST_F(FilterTest, filterTestValidInput)
{
    rv = filterROI(cloud.makeShared(), filteredCloud, 3.0, 3.0, 0.0, 1.0, -1.0);
    EXPECT_TRUE(rv);
}

TEST_F(FilterTest, filterTestFilteringNoOffset)
{
    rv = filterROI(cloud.makeShared(), filteredCloud, 1.0, 1.0, 0.0, 1.0, -1.0);
    EXPECT_TRUE(rv);
    EXPECT_EQ(filteredCloud.size(), 2);
}

TEST_F(FilterTest, filterTestFilteringWithOffset)
{
    rv = filterROI(offsetCloud.makeShared(), filteredCloud, 1.0, 1.0, 5.0, 1.0, -1.0);
    EXPECT_TRUE(rv);
    EXPECT_EQ(filteredCloud.size(), 2);
}

TEST_F(FilterTest, filterTestFilteringWithOffsetForApp)
{
    rv = filterROI(appOffsetCloud.makeShared(), filteredCloud, 10, 2.0, 9.0, 1.0, -1.0);
    EXPECT_TRUE(rv);
    EXPECT_EQ(filteredCloud.size(), 2);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_stopping_distance_monitor_test");
    ::testing::InitGoogleTest(&argc, argv);

    ros::shutdown();
    return RUN_ALL_TESTS();
}