#include "lidar_stopping_distance_monitor/lidar_stopping_distance_monitor.h"

LidarStoppingDistanceMonitor::LidarStoppingDistanceMonitor() : m_outputCoordinateFrame(""),
                                                               m_laneWidth(0.0),
                                                               m_roiLen(0.0),
                                                               m_maxH(0.0),
                                                               m_minH(0.0),
                                                               m_maxVehicleDeceleration(0.0)
{
    m_nodeHandle.getParam("/lidar_stopping_distance_monitor/lane_width", m_laneWidth);
    m_nodeHandle.getParam("/lidar_stopping_distance_monitor/roi_length", m_roiLen);
    m_nodeHandle.getParam("/lidar_stopping_distance_monitor/max_height", m_maxH);
    m_nodeHandle.getParam("/lidar_stopping_distance_monitor/min_height", m_minH);
    m_nodeHandle.getParam("/lidar_stopping_distance_monitor/max_vehicle_deceleration", m_maxVehicleDeceleration);
    m_nodeHandle.getParam("/lidar_stopping_distance_monitor/output_coordinate_frame",
                                   m_outputCoordinateFrame);

    ROS_INFO("Value of params:");
    ROS_INFO("Output coordinate frame: %s", m_outputCoordinateFrame.c_str());
    ROS_INFO("Lane width: %f", m_laneWidth);
    ROS_INFO("ROI len: %f", m_roiLen);
    ROS_INFO("Max H: %f", m_maxH);
    ROS_INFO("Min H: %f", m_minH);
    ROS_INFO("Max deceleration: %f", m_maxVehicleDeceleration);
}

bool LidarStoppingDistanceMonitor::filterROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
                pcl::PointCloud<pcl::PointXYZ>& outputCloud,
                const double length,
                const double width,
                const double offsetX,
                const double maxH,
                const double minH)
{
    if(length < 0 || width < 0)
    {
        return false;
    }

    // Comparison points
    double x1 = offsetX - length / 2 - Treshold;
    double x2 = offsetX + length / 2 + Treshold;
    double y1 = -width / 2 - Treshold;
    double y2 = width / 2 + Treshold;

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr 
                rangeCondition(new pcl::ConditionAnd<pcl::PointXYZ>);
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, x1)
            ));
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, x2)
            ));
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, y1)
            ));
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, y2)
            ));
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, minH - Treshold)
            ));
    rangeCondition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
            new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, maxH + Treshold)
            ));

    pcl::ConditionalRemoval<pcl::PointXYZ> conditionalRemover;
    conditionalRemover.setCondition(rangeCondition);
    conditionalRemover.setInputCloud(inputCloud);
    conditionalRemover.setKeepOrganized(false);
    conditionalRemover.filter(outputCloud);

    return true;
}