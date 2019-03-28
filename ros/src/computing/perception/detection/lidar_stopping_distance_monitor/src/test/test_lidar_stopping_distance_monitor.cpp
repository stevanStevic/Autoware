#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(ExampleCase, exampleFail)
{
    EXPECT_TRUE(false);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_stopping_distance_monitor_test");
    ::testing::InitGoogleTest(&argc, argv);

    ros::shutdown();
    return RUN_ALL_TESTS();
}