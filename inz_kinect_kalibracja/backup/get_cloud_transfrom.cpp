#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Program już nie używany w pracy
// Program do utowrzenia obróceonego układu współrzędnych i publikowania tf z sl_base do tego układu
int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_cloud_transform");
    ros::NodeHandle n;

    tf::TransformBroadcaster br;   
    tf::TransformListener listener;
    tf::Transform new_rgb_frame;
    tf::Transform new_kinect_frame;
    tf::StampedTransform cloud_transform;

    new_rgb_frame.setOrigin(tf::Vector3(-0.8679874110393013, 0.07934434818709263, 1.0186573965812087));
    tf::Quaternion qt1(-0.7238872332816215, 0.19634739870117968, -0.1835996946415743, 0.6353944638117962);
    //qt1.setRPY(-3.14/2, 0, 3.14);
    new_rgb_frame.setRotation(qt1);

    new_kinect_frame.setOrigin(tf::Vector3(0,0,0));
    tf::Quaternion qt(0,0,0,1);
    qt.setRPY(0, 0, 3.14/2);
    new_kinect_frame.setRotation(qt);

    cloud_transform.mult(new_rgb_frame, new_kinect_frame);

    br.sendTransform(tf::StampedTransform(new_rgb_frame, ros::Time::now(), "sl_base", "new_rgb_frame"));
    br.sendTransform(tf::StampedTransform(new_kinect_frame, ros::Time::now(), "new_rgb_frame", "new_kinect_frame"));
    //br.sendTransform(tf::StampedTransform(cloud_transform, ros::Time::now(), "sl_base", "test"));  

    // Wypisanie wartości na konsoli
    std::cout << "Translation: " << cloud_transform.getOrigin().getX() << " " << cloud_transform.getOrigin().getY() << " " << cloud_transform.getOrigin().getZ() << std::endl;
    std::cout << "Rotation: " << cloud_transform.getRotation().getX() << " " << cloud_transform.getRotation().getY() << " " << cloud_transform.getRotation().getZ() << " " << cloud_transform.getRotation().getW() << std::endl;

    ros::Rate rate(30);
    while(ros::ok)
    {
        br.sendTransform(tf::StampedTransform(new_rgb_frame, ros::Time::now(), "sl_base", "new_rgb_frame"));
        br.sendTransform(tf::StampedTransform(new_kinect_frame, ros::Time::now(), "new_rgb_frame", "new_kinect_frame"));
        br.sendTransform(tf::StampedTransform(cloud_transform, ros::Time::now(), "sl_base", "kinect_frame"));
        rate.sleep();
    }

    return 0;
}