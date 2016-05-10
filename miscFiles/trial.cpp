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

void
cloud_cb (const sensor_msgs::PointCloud2 cloud_msg)
{
  //ALTERNATIVE CONVERSION FROM POINTCLOUD2 TO PCL
  /*pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::fromPCLPointCloud2(*cloud,*cloud_filtered2);

  std::cout << "PointCloud after filtering: " << cloud_filtered2->width * cloud_filtered2->height << " data points." << std::endl;
*/
  std::cout << cloud_msg.header.frame_id << std::endl;
  pcl::PointCloud<pcl::PointXYZ> cloud3;
  pcl::fromROSMsg (cloud_msg, cloud3);

  std::cout << cloud3.header.frame_id << std::endl;


  //pcl::io::savePCDFileASCII ("circle_found.pcd", cloud3);

  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold (0.01);
  seg.setRadiusLimits(0.05,0.12);

  seg.setInputCloud (cloud3.makeShared());
  seg.segment (inliers, coefficients);

  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(coefficients, ros_coefficients);

  pub.publish(ros_coefficients);

  //std::cout<< ros_coefficients;

  pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud<pcl::PointXYZ>(cloud3, inliers, *final);

  pcl::io::savePCDFileASCII ("circle_found.pcd", *final);


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

  /*ros::Time time = ros::Time::now();

  tf::TransformListener listener(ros::Duration(10.0));*/

  /*try{
    listener.waitForTransform("/torso", cloud_msg->header.frame_id, time, ros::Duration(10.0));
    //listener.lookupTransform("/torso", frame, time, transform);
    listener.transformPoint("/torso", pt, pt_transformed);

  }
  catch (tf::TransformException ex){
       ROS_ERROR("%s",ex.what());
  }*/
  //pt_transformed = pt.transform(transform);
  //pt_transformed = transform * pt;

  //std::cout << pt_transformed.point.x << ", " << pt_transformed.point.y << ", " <<
  //             pt_transformed.point.z << "\n" << std::endl;

  pointPub.publish(pt);
  //TRANSFORM CLOUD FROM KINECT POSITION TO TORSO TRANSFORM
  /*^CAt time 1454592251.347
- At time 1454594654.484
- Translation: [-0.080, 0.371, -0.073]
- Rotation: in Quaternion [0.647, -0.638, 0.298, 0.291]
            in RPY (radian) [-3.136, -0.861, -1.560]
            in RPY (degree) [-179.665, -49.304, -89.387]*/

  /*std::cout << ros_coefficients.values[0] << ", ";
  std::cout << ros_coefficients.values[1] << ", ";
  std::cout << ros_coefficients.values[2] << "\n\n";*/

   //Eigen::Quaternionf rotationMat(-0.291, 0.647, -0.638, 0.298);

  // Eigen::Vector3f rpy(-3.136, -0.861, -1.560);

   //Eigen::Vector3f rpy(-2.281, -0.014, -1.564);
   /*Eigen::AngleAxisf roll(rpy[0], Eigen::Vector3f::UnitX());
   Eigen::AngleAxisf pitch(rpy[1], Eigen::Vector3f::UnitY());
   Eigen::AngleAxisf yaw(rpy[2], Eigen::Vector3f::UnitZ());*/

   /*Eigen::Affine3d r = create_rotation_matrix(-3.136, -0.861, -1.560);

   Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(-0.080, 0.371, -0.073)));
   Eigen::Matrix4d m = (t * r).matrix(); // Option 1

   Eigen::Vector4d centre(ros_coefficients.values[0],ros_coefficients.values[1],ros_coefficients.values[2],0);
   Eigen::Vector4d result = m * centre;


   pcl::PointCloud<pcl::PointXYZRGB>::Ptr final(new pcl::PointCloud<pcl::PointXYZRGB>);

 	 pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud, inliers, *final);

   // Executing the transformation
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  // You can either apply transform_1 or transform_2; they are the same
   pcl::transformPointCloud (*cloud, *transformed_cloud, m);

   pcl::io::savePCDFileASCII ("circle_found.pcd", *final);
   pcl::io::savePCDFileASCII ("test_pcd.pcd", *transformed_cloud);


   std::cout << "Saved " << inliers.indices.size() <<
   " data points to test_pcd.pcd\n";*/



   /*Eigen::Quaternionf q(roll * yaw * pitch);

   Eigen::Matrix3f mat = q.matrix();


   /*Eigen::Matrix3f rot = rotationMat.toRotationMatrix();
   Eigen::Matrix4f transform2;
   transform2 << rot(0,0), rot(0,1), rot(0,2), 0.298,
                rot(1,0), rot(1,1), rot(1,2), -0.075,
                rot(2,0), rot(2,1), rot(2,2), 0.235,
                0, 0, 0, 1;
   Eigen::Matrix4f totalRot = transform*transform2;

   - Translation: [0.298, -0.075, 0.235]
- Rotation: in Quaternion [0.647, -0.638, 0.298, -0.291]
            in RPY (radian) [-2.281, -0.014, -1.564]
            in RPY (degree) [-130.700, -0.800, -89.600]*/

   //Eigen::Vector3f v(ros_coefficients.values[0]-0.080,ros_coefficients.values[1]+0.371,ros_coefficients.values[2]-0.073);
   //Eigen::Vector3f v(ros_coefficients.values[0]+0.298,ros_coefficients.values[1]-0.075,ros_coefficients.values[2]+0.235);

   //std::cout << v << "\n";
   //Eigen::Vector3f rotPoint = mat*v;
   //std::cout << result << "\n\n";



}



int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output model coefficients
  pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

  pointPub = nh.advertise<geometry_msgs::PointStamped> ("outputPoint", 1);

  // Spin
  ros::spin ();
}
