#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

void point_cb(geometry_msgs::PointStamped point_msg) {
  //sstd::cout << point_msg.point.x << std::endl;
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "move_to_bowl");
  ros::NodeHandle nh;

  // Subscribe to the table objects PointCloud2 from the kinect
  ros::Subscriber sub = nh.subscribe ("/bowlPos", 1, point_cb);


  // Spin
  ros::spin ();
}
