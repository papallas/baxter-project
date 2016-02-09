#include <ros/ros.h>
// PCL specific includes
#include <pcl/sample_consensus/sac_model_circle.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/PointStamped.h>

ros::Publisher pub;
ros::Publisher pointPub;

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rz * ry * rx;
}

// Create a callback method on the receipt of a PointCloud2 object
void
cloud_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
  std::cout << cloud_msg.header.frame_id << std::endl;

  // Convert the PointCloud2 to a PCL cloud
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg (cloud_msg, pcl_cloud);

  std::cout << pcl_cloud.header.frame_id << std::endl;

  // Save PCL cloud as an ASCII file
  //pcl::io::savePCDFileASCII ("circle_found.pcd", pcl_cloud);

  // Create model variables to store the circle model values in
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  // Using RANSAC to find a circle within a 2D space
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  // Max distance of 1 cm between points
  seg.setDistanceThreshold (0.01);
  // Set limit on size of bowl - not search too big or too small circles
  seg.setRadiusLimits(0.05,0.12);

  // Set the PCL cloud as the input and segment into the inliers/coefficients
  seg.setInputCloud (pcl_cloud.makeShared());
  seg.segment (inliers, coefficients);

  // Convert the model coefficients to a set of publishable coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);

  // Publish on output
  pub.publish(ros_coefficients);

  // Print coefficients to terminal
  //std::cout<< ros_coefficients;

  // Initialise new point cloud of just bowl rim pointcloud and output to text
  // file
  pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud<pcl::PointXYZ>(pcl_cloud, inliers, *final);
  pcl::io::savePCDFileASCII ("circle_found.pcd", *final);

  //Using a transform listener, create a point using the coefficients
  tf::TransformListener listener;
  tf::StampedTransform transform;

  geometry_msgs::PointStamped pt;
  geometry_msgs::PointStamped pt_transformed;
  pt.header = cloud_msg.header;
  pt.header.frame_id = cloud_msg.header.frame_id;
  pt.header.stamp = ros::Time();
  pt.point.x = ros_coefficients.values[0];
  pt.point.y = ros_coefficients.values[1];
  pt.point.z = ros_coefficients.values[2];

  // Publish the created point for viewing in rViz
  pointPub.publish(pt);
}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Subscribe to the table objects PointCloud2 from the kinect
  ros::Subscriber sub = nh.subscribe ("/tabletop_pointcloud_1", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  pointPub = nh.advertise<geometry_msgs::PointStamped> ("outputPoint", 1);

  // Spin
  ros::spin ();
}
