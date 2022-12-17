#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include <ros/package.h>


 std_msgs::Bool collisionDetected;
 std_msgs::Float32 vel;

//// Program sprawdzający czy w obszarze roboczym robota znajduję sie człowiek

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    vel.data = 1.0; // skalowanie prędkości
    collisionDetected.data = false; // wynik detekcji kolizji
    int ile = 0; // Zmienna zliczająca

    for (int i = 0; i < temp_cloud->points.size(); i++) // Sprawdzenie czy punkty z chmury znajdują się w okręgu o promieniu 1m (w przybliżeniu obszar roboczy robota)
    {
        if (temp_cloud->points[i].x < 1.0  &&  temp_cloud->points[i].y < 1.0  && temp_cloud->points[i].z< 1.0)
            ile ++;
    }

    if (ile > 10) // Jeśli 10 punktów znajduję sie w przeszrzeniu robocznej 
    {   ROS_WARN("Czlowiek w przesztrzeni roboczej");
        collisionDetected.data = true; // flaga kolizji = true
        vel.data = 0.5; // ograniczenie prędkości
    }else{
        ROS_INFO("Brak czlowieka w przestrzeni roboczej");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "colision_detection");
    ros::NodeHandle n;

    ros::Subscriber point_cloud_sub = n.subscribe("/kinect2/hd/points", 100, point_cloud_callback); // subrcriber chmury punktów
    ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("/es_trajectory/time_scalling", 1000); // publisher sklaowanie prędkości robota
    ros::Publisher col_flag_pub = n.advertise<std_msgs::Bool>("/colison_flag", 1000); // publisher detekcji kolizji
        
    ros::Rate loop_rate(20);

    while(ros::ok)
    {
        loop_rate.sleep();
        ros::spinOnce();
        col_flag_pub.publish(collisionDetected);
        vel_pub.publish(vel);
        
    }

    return 0;
}