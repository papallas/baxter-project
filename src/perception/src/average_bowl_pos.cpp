#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

ros::Publisher pub;

int counter = 0;

// Create vectors to store an average bowl centre over 20 frames
std::vector<float> xVals(20,0.0);
std::vector<float> yVals(20,0.0);;
std::vector<float> zVals(20,0.0);;

// Create a callback method on the receipt of the bowl pointcloud
void
point_cb (const geometry_msgs::PointStamped point_msg)
{
    // Count the frame number with variables
    float frameCount = 20.0;
    int frameInt = 20;
    // Before 20 frames have been received
    if (counter < frameInt) {
        // Create the vectors with xyz values from 20 frames
        xVals[counter] = point_msg.point.x;
        yVals[counter] = point_msg.point.y;
        zVals[counter] = point_msg.point.z;

        counter++;
    }
    // If 20 frames have passed, start to publish
    if (counter == frameInt) {
        // Get rid of the first value and add the most current
        // frame to the end to get a rolling average
        xVals.erase(xVals.begin());
        xVals.push_back(point_msg.point.x);
        yVals.erase(yVals.begin());
        yVals.push_back(point_msg.point.y);
        zVals.erase(zVals.begin());
        zVals.push_back(point_msg.point.z);

        float averageX = 0;
        float averageY = 0;
        float averageZ = 0;
        
        // Iterate over the vector and get the average xyz values
        for(std::vector<float>::iterator it = xVals.begin(); it != xVals.end(); ++it)
          averageX += *it;
        for(std::vector<float>::iterator it = yVals.begin(); it != yVals.end(); ++it)
          averageY += *it;
        for(std::vector<float>::iterator it = zVals.begin(); it != zVals.end(); ++it)
          averageZ += *it;
    
        averageX /= frameCount;
        averageY /= frameCount;
        averageZ /= frameCount;

        // Create a pointstamped object from the value and publish
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

// Main function
int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "average_bowl");
    ros::NodeHandle nh;

    // Subscribe to the table objects PointCloud2 from the kinect
    ros::Subscriber sub = nh.subscribe ("/initialBowlPos", 1, point_cb);

    // Publish the bowl centre average
    pub = nh.advertise<geometry_msgs::PointStamped> ("bowlAverage", 1);

    // Spin
    ros::spin ();
}
