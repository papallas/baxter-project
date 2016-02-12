#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

ros::Publisher pub;

int counter = 0;

std::vector<float> xVals(20,0.0);
std::vector<float> yVals(20,0.0);;
std::vector<float> zVals(20,0.0);;

// Create a callback method on the receipt of a PointCloud2 object
void
point_cb (const geometry_msgs::PointStamped point_msg)
{
  if (counter < 20) {
    xVals[counter] = point_msg.point.x;
    yVals[counter] = point_msg.point.y;
    zVals[counter] = point_msg.point.z;

    counter++;
  }

  if (counter == 20) {
    xVals.erase(xVals.begin());
    xVals.push_back(point_msg.point.x);
    yVals.erase(yVals.begin());
    yVals.push_back(point_msg.point.y);
    zVals.erase(zVals.begin());
    zVals.push_back(point_msg.point.z);

    float averageX = 0;
    float averageY = 0;
    float averageZ = 0;

    for(std::vector<float>::iterator it = xVals.begin(); it != xVals.end(); ++it)
      averageX += *it;
    for(std::vector<float>::iterator it = yVals.begin(); it != yVals.end(); ++it)
      averageY += *it;
    for(std::vector<float>::iterator it = zVals.begin(); it != zVals.end(); ++it)
      averageZ += *it;

    averageX /= 20.0;
    averageY /= 20.0;
    averageZ /= 20.0;

    geometry_msgs::PointStamped pt;
    pt.header = point_msg.header;
    pt.header.frame_id = point_msg.header.frame_id;
    pt.header.stamp = ros::Time();
    pt.point.x = averageX;
    pt.point.y = averageY;
    pt.point.z = averageZ;

    pub.publish(pt);
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
