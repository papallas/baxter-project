#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pub;

int counter = 0;
float averageX = 0;
float averageY = 0;
float averageZ = 0;

// Create a callback method on the receipt of a PointCloud2 object
void
point_cb (const geometry_msgs::PointStamped point_msg)
{
  if (counter < 20) {
    averageX += point_msg.point.x;
    averageY += point_msg.point.y;
    averageZ += point_msg.point.z;
    counter++;
  }

  if (counter == 20) {
    averageX /= 20.0;
    averageY /= 20.0;
    averageZ /= 20.0;
  }

  if (counter >= 20) {
    geometry_msgs::PointStamped pt;
    pt.header = point_msg.header;
    pt.header.frame_id = point_msg.header.frame_id;
    pt.header.stamp = ros::Time();
    pt.point.x = averageX;
    pt.point.y = averageY;
    pt.point.z = averageZ;

    pub.publish(pt);

    counter++;
  }

  if (counter > 25) {
    counter = 0;
  }
}


int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "find_bowl");
  ros::NodeHandle nh;

  // Subscribe to the table objects PointCloud2 from the kinect
  ros::Subscriber sub = nh.subscribe ("/initialBowlPos", 1, point_cb);

  pub = nh.advertise<geometry_msgs::PointStamped> ("bowlPos", 1);

  // Spin
  ros::spin ();
}
