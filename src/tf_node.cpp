#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_node");
    ros::NodeHandle node;
    ROS_INFO("TF node started");
    // Init tf broadcaster and tf listener
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    listener.waitForTransform("/kinect2_link", "/kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(4.0));
    // Start loop
    while (node.ok())
    {
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/kinect2_link", "/kinect2_rgb_optical_frame", ros::Time(0), ros::Duration(4.0));
            listener.lookupTransform("/kinect2_link", "/kinect2_rgb_optical_frame", ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        // Broadcast renamed tf
        broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/kinect2_link", "/multispectral_frame"));
    }
    // Shut down
    ROS_INFO("TF node shutted down");
    return 0;
};