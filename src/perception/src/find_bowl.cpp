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
pcl_msgs::ModelCoefficients set_coefficients;

// Function to cluster pointcloud into separate objects
std::vector <pcl::PointIndices> getObjectClusters(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    // Search over the entire pointcloud
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
    // Estimate normals
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud.makeShared());
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    // Set filter limits for the indices
    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud.makeShared());
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    // Use the region growing algorithm with tested variables
    // The thresholds and cluster sizes are explained further
    // in the report
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setMinClusterSize (2000);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud.makeShared());
    reg.setIndices (indices);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (20.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);

    // Extract the clusters into a new pointcloud
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    // SHOW POINTCLOUD OF SEGMENTED REGIONS
    /*pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud (colored_cloud);
    while (!viewer.wasStopped ())
    {
      boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }*/
    return clusters;
}

// Function to retrieve only light coloured/white objects
pcl::PointCloud<pcl::PointXYZRGB> getWhiteObjectCloud(std::vector <pcl::PointIndices> clusters,
                pcl::PointCloud<pcl::PointXYZRGB> coloured_cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> whiteCloud;
    // Iterate over the separated clusters
    for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
    {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    float R = 0;
    float G = 0;
    float B = 0;
    // When looking at the cluster, loop over the points
    // and calculate the total R, G and B values of each point
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
       cloud_cluster->points.push_back (coloured_cloud.points[*pit]);
       R += float(coloured_cloud.points[*pit].r);
       G += float(coloured_cloud.points[*pit].g);
       B += float(coloured_cloud.points[*pit].b);
    }
    // Get the average overall RGB colour for the cluster
    R /= cloud_cluster->points.size();
    G /= cloud_cluster->points.size();
    B /= cloud_cluster->points.size();
    float average = (R + G + B)/3.0;
    // Use determined threshold white values to test
    if (R > 150 && G > 150 && B > 150) {
      pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_cluster, it->indices, whiteCloud);
    }
    }
    // View white objects in pointcloud viewer
    /*pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(whiteCloud.makeShared());
    while (!viewer.wasStopped ())
    {}*/
    return whiteCloud;
}

// Get the bowl in the cloud by finding the rim
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
   seg.setDistanceThreshold (0.2);
   // Set limit on size of bowl - not search too big or too small circles
   seg.setRadiusLimits(0.05,0.12);

   // Set the PCL cloud as the input and segment into the inliers/coefficients
   seg.setInputCloud (whiteCloud.makeShared());
   if (whiteCloud.points.size() != 0) {
     seg.segment (inliers, coefficients);

     // Convert the model coefficients to a set of publishable coefficients
     pcl_msgs::ModelCoefficients ros_coefficients;
     pcl_conversions::fromPCL(coefficients, ros_coefficients);

     set_coefficients = ros_coefficients;

     return ros_coefficients;
   }
   else {
     return set_coefficients;
   }
}

// Create a callback method on the receipt of a PointCloud2 object
// from the Kinect
void cloud_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
    // Convert the PointCloud2 to a PCL cloud
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg (cloud_msg, pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB> pcl_color;
    pcl::fromROSMsg (cloud_msg, pcl_color);

    // Call the main functions on the coloured cloud: object
    // clustering, white separation, rim detection
    std::vector <pcl::PointIndices> clusters = getObjectClusters(pcl_color);

    pcl::PointCloud<pcl::PointXYZRGB> whiteCloud = getWhiteObjectCloud(clusters,
                pcl_color);

    pcl_msgs::ModelCoefficients bowl_coefficients = getBowlInCloud(whiteCloud);

    // Create a pointstamped object from the value and publish
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

// Main function
int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "find_bowl");
    ros::NodeHandle nh;

    // Subscribe to the table objects PointCloud2 from the kinect
    ros::Subscriber sub = nh.subscribe ("/tabletop_pointcloud_1", 1, cloud_cb);
    // Publish the detected bowl centre
    pointPub = nh.advertise<geometry_msgs::PointStamped> ("initialBowlPos", 1);

    // Spin
    ros::spin ();
}
