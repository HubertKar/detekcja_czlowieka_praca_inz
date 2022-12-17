#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include <ros/package.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "nav_msgs/Odometry.h"

#include "fcl/shape/geometric_shapes.h"
#include "fcl/narrowphase/narrowphase.h"
#include "fcl/collision.h"
#include "fcl/ccd/motion.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/octree.h"
#include "fcl/traversal/traversal_node_octree.h"
#include "fcl/broadphase/broadphase.h"
#include "fcl/shape/geometric_shape_to_BVH_model.h"
#include "fcl/math/transform.h"

octomap_msgs::Octomap octomap1;


////////////////// Detekcja kolizji z wykorzystaniem octree, konwersja z chmury punktów na octree (nie skończona bo nie wiem jak to zrobić)
void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    // Konwersja z sensor_msgs::PointCloud2 na pcl::PointCloud<pcl::PointXYZ> można zamienić z pointCloud2 na pointCloud wtedy konwersja nie powinna być potrzebna
    // współrzędne pounktów w PointCloud2  są zapisane w taki sposób że nie do nich w prosty sposób dostać
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    double res = 1.0;
    octomap::OcTree* octTree = new octomap::OcTree(res);
    octomap::Pointcloud octPointCloud;

    for (int i = 0; i < temp_cloud->points.size(); i++)
    {
        octomap::point3d endpoint((float) temp_cloud->points[i].x, (float) temp_cloud->points[i].y, (float) temp_cloud->points[i].z);
        octPointCloud.push_back(endpoint);
    }

    octomap::point3d origin(0.0,0.0,0.0);
    octTree->insertPointCloud(octPointCloud, origin);
    octTree->updateInnerOccupancy();


    //octomap::OcTree* tree2 = new octomap::OcTree(<const octomap::OcTree>(octTree));
    //octomap::OcTree* tree2 = new octomap::OcTree(octTree);

    octomap1.binary = 1;
    octomap1.id = 1;
    octomap1.resolution =0.1;
    octomap1.header.frame_id = "world1";
    octomap1.header.stamp = ros::Time::now();

    bool r = octomap_msgs::fullMapToMsg(*octTree, octomap1);

    std::vector<fcl::CollisionObject*> boxes;
    //fcl::generateBoxesFromOctomap(boxes, *tree2);
    bool collisionDetected = false;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "colision_detection");
    ros::NodeHandle n;
    pcl::PointCloud<pcl::PointXYZ> point_cloud;

    ros::Publisher octo_map_pub = n.advertise<octomap_msgs::Octomap>("LaserOctmap", 1);
    ros::Subscriber point_cloud_sub = n.subscribe("/kinect2/hd/points", 100, point_cloud_callback);
    ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("/es_trajectory/time_scalling", 1000);
    ros::Publisher col_flag_pub = n.advertise<std_msgs::Bool>("/colison_flag", 1000);
    
    
    ros::Rate loop_rate(20);

    while(ros::ok)
    {
    loop_rate.sleep();
    ros::spinOnce();
    octo_map_pub.publish(octomap1);
    }

    return 0;
}