#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

tf::Transform transform_TCP;
void robot_tf_Callback(const geometry_msgs::Pose::ConstPtr& msg)
{
    transform_TCP.setOrigin(tf::Vector3(msg->position.x, msg->position.y, msg->position.z));
    transform_TCP.setRotation( tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w));
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalibracja");
    ros::NodeHandle n;
    ros::Rate rate(30);
    ros::Subscriber sub = n.subscribe("/es_arm/cartesian_pose", 100, robot_tf_Callback);

    tf::Transform transform_image_frame;
    tf::TransformBroadcaster br;

    //Kinect image frame
    transform_image_frame.setOrigin(tf::Vector3(-0.8679874110393013, 0.07934434818709263, 1.0186573965812087));
    tf::Quaternion qt1(-0.7238872332816215, 0.19634739870117968, -0.1835996946415743, 0.6353944638117962);
    transform_image_frame.setRotation(qt1);

    while (ros::ok)
    {
        br.sendTransform(tf::StampedTransform(transform_TCP, ros::Time::now(), "sl_base", "effector_frame")); // publikuj tf efektora
        br.sendTransform(tf::StampedTransform(transform_image_frame, ros::Time::now(), "sl_base", "kinect2_rgb_optical_frame"));
        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}