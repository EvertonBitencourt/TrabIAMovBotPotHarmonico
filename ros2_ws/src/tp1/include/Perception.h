#ifndef PERCEPTION_H
#define PERCEPTION_H

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

class Perception
{
    public:
        Perception();
        
        std::vector<float> getLatestLaserRanges();
        std::vector<float> getLatestSonarRanges();
        std::vector<float> getLatestPose();
        
        void receiveLaser(const sensor_msgs::msg::LaserScan::ConstSharedPtr &value);
        void receiveSonar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &value);
        void recievePose(const nav_msgs::msg::Odometry::ConstSharedPtr &value);

    private:
        sensor_msgs::msg::LaserScan laserROS;
        sensor_msgs::msg::PointCloud2 sonarROS;
        nav_msgs::msg::Odometry poseROS;

};

#endif // PERCEPTION_H