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

#include <geometry_msgs/PointStamped.h>

#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

ros::Publisher pointPub;

std::vector <pcl::PointIndices> getObjectClusters(pcl::PointCloud<pcl::PointXYZ> cloud)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud.makeShared());
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (200);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud.makeShared());
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (20.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    return clusters;
}

std::vector <pcl::PointIndices> getColourClusters(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud.makeShared());
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (10);
    reg.setPointColorThreshold (6);
    reg.setRegionColorThreshold (5);
    reg.setMinClusterSize (600);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    return clusters;
}

pcl::PointCloud<pcl::PointXYZRGB> getWhiteObjectCloud(std::vector <pcl::PointIndices> clusters,
                pcl::PointCloud<pcl::PointXYZRGB> coloured_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB> whiteCloud;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    float R = 0;
    float G = 0;
    float B = 0;
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
       cloud_cluster->points.push_back (coloured_cloud.points[*pit]);
       R += float(coloured_cloud.points[*pit].r);
       G += float(coloured_cloud.points[*pit].g);
       B += float(coloured_cloud.points[*pit].b);
    }
    R /= cloud_cluster->points.size();
    G /= cloud_cluster->points.size();
    B /= cloud_cluster->points.size();
    float average = (R + G + B)/3.0;
    //std::cout << "Average total = " << average << std::endl;
    // IF OBJECT IS WHITE/NEAR WHITE
    if (average > 180) {
      pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_cluster, it->indices, whiteCloud);
    }
  }

  return whiteCloud;
}

pcl_msgs::ModelCoefficients getBowlInCloud(pcl::PointCloud<pcl::PointXYZRGB> whiteCloud)
{
  // Create model variables to store the circle model values in
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  seg.setOptimizeCoefficients (true);
  // Using RANSAC to find a circle within a 2D space
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  // Max distance of 1 cm between points
  seg.setDistanceThreshold (0.03);
  // Set limit on size of bowl - not search too big or too small circles
  seg.setRadiusLimits(0.05,0.12);

  // Set the PCL cloud as the input and segment into the inliers/coefficients
  seg.setInputCloud (whiteCloud.makeShared());
  seg.segment (inliers, coefficients);

  // Convert the model coefficients to a set of publishable coefficients
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);

  return ros_coefficients;
}

// Create a callback method on the receipt of a PointCloud2 object
void
cloud_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
  // Convert the PointCloud2 to a PCL cloud
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl::fromROSMsg (cloud_msg, pcl_cloud);

  pcl::PointCloud<pcl::PointXYZRGB> pcl_color;
  pcl::fromROSMsg (cloud_msg, pcl_color);

  std::vector <pcl::PointIndices> clusters = getColourClusters(pcl_color);

  pcl::PointCloud<pcl::PointXYZRGB> whiteCloud = getWhiteObjectCloud(clusters,
                pcl_color);

  pcl_msgs::ModelCoefficients bowl_coefficients = getBowlInCloud(whiteCloud);

  geometry_msgs::PointStamped pt;
  geometry_msgs::PointStamped pt_transformed;
  pt.header = cloud_msg.header;
  pt.header.frame_id = cloud_msg.header.frame_id;
  pt.header.stamp = ros::Time();
  pt.point.x = bowl_coefficients.values[0];
  pt.point.y = bowl_coefficients.values[1];
  pt.point.z = bowl_coefficients.values[2];

  // Publish the created point for viewing in rViz
  pointPub.publish(pt);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "find_bowl");
  ros::NodeHandle nh;

  // Subscribe to the table objects PointCloud2 from the kinect
  ros::Subscriber sub = nh.subscribe ("/tabletop_pointcloud_1", 1, cloud_cb);

  pointPub = nh.advertise<geometry_msgs::PointStamped> ("initialBowlPos", 1);

  // Spin
  ros::spin ();
}
