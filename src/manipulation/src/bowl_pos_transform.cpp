#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <manipulation/RequestBowlPos.h>

ros::Publisher pointPub;
geometry_msgs::PointStamped baxterBowl;

void point_cb(geometry_msgs::PointStamped point_msg) {

  tf::TransformListener listener;
  tf::StampedTransform transform;

  try{
      listener.waitForTransform("kinect2_1_ir_optical_frame", "torso", point_msg.header.stamp, ros::Duration(3.0));
      listener.lookupTransform("kinect2_1_ir_optical_frame", "torso", point_msg.header.stamp, transform);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
  }
  listener.transformPoint("torso", point_msg, baxterBowl);

  pointPub.publish(baxterBowl);
}

bool add(manipulation::RequestBowlPos::Request &req,
         manipulation::RequestBowlPos::Response &res)
{
  res.x = baxterBowl.point.x;
  res.y = baxterBowl.point.y;
  res.z = baxterBowl.point.z;
  return true;
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "move_to_bowl");
  ros::NodeHandle nh;

  // Subscribe to the table objects PointCloud2 from the kinect
  ros::Subscriber sub = nh.subscribe ("/bowlPos", 1, point_cb);

  pointPub = nh.advertise<geometry_msgs::PointStamped> ("scooppos", 1);

  ros::ServiceServer service = nh.advertiseService("bowl_pos_req", add);

  // Spin
  ros::spin ();
}
