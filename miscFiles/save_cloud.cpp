#include <ros/ros.h>
#include "std_msgs/String.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

void
cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(*msg, cloud);
    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

    std::cout << "blah" << "\n";

    std::cout << "Saved " << cloud.points.size () <<
    " data points to test_pcd.pcd." << std::endl;
}

int
main (int argc, char** argv)
{
    ros::init(argc, argv, "pcd_write_topic");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("input", 1, cloud_cb);
    ros::spin();
}
