#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <manipulation/RequestBowlPos.h>

ros::Publisher pointPub;
geometry_msgs::PointStamped baxterBowl;

// On the receipt of the cumulative average bowl centre
void point_cb(geometry_msgs::PointStamped point_msg) {

    // Listen for the transform between the kinect and the torso
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
    // Transform the received point and publish it
    listener.transformPoint("torso", point_msg, baxterBowl);

    pointPub.publish(baxterBowl);
}

// After the call is made from the shop node, respond with the
// current average bowl position
bool add(manipulation::RequestBowlPos::Request &req,
         manipulation::RequestBowlPos::Response &res)
{
    res.x = baxterBowl.point.x;
    res.y = baxterBowl.point.y;
    res.z = baxterBowl.point.z;
    return true;
}

// Main function
int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "bowl_transform");
    ros::NodeHandle nh;

    // Subscribe to the cumulatively averaged bowl position
    ros::Subscriber sub = nh.subscribe ("/bowlPos", 1, point_cb);

    // Publisher for the actual position in Baxter coordinates
    pointPub = nh.advertise<geometry_msgs::PointStamped> ("scooppos", 1);

    // Service to communicate with the main shopkeeper node
    ros::ServiceServer service = nh.advertiseService("bowl_pos_req", add);

    // Spin
    ros::spin ();
}
