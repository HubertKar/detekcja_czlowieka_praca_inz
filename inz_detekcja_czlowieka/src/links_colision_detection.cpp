#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include "ros/ros.h"
#include <ros/package.h>

#include "fcl/shape/geometric_shapes.h"
#include "fcl/collision.h"
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <Eigen/Eigen>
#include "fcl/math/transform.h"

#include "geometry_msgs/Pose.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

std_msgs::Float32 time_scalling;
float previous_value = 1.0;
int no_human_counter = 0;

int check_collision(const sensor_msgs::PointCloud2::ConstPtr& msg, double workspace_size, double box_size, int prog1, int prog2)
{
    int colliosion_result = 0; // zmienna na stan kolziji 0 - brak kolziji, 1 - obszar roboczy, 2 - kolizja z robotem
    time_scalling.data = 1.0; // skalowanie czasu wykonywania ruchu (ograniczenie prędkości)

    /////////////////// Część związana z robotem /////////
    tf::TransformBroadcaster br;
    tf::TransformListener listener;

    tf::StampedTransform transform1;
    tf::StampedTransform transform2;
    tf::StampedTransform transform3;
    tf::StampedTransform transform4;

    listener.waitForTransform("sl_base", "sl_1", ros::Time(0), ros::Duration(1.0)); // Czekanie na transformate
    listener.lookupTransform("sl_base", "sl_1", ros::Time(0), transform1); // Odczytanie transformaty
    tf::Vector3 t1 = transform1.getOrigin(); // Przesuniecie w osi z
    t1.setZ(t1.getZ() - 0.08);
    transform1.setOrigin(t1);

    listener.waitForTransform("sl_base", "sl_2", ros::Time(0), ros::Duration(1.0)); 
    listener.lookupTransform("sl_base", "sl_2", ros::Time(0), transform2); 
    tf::Vector3 t2 = transform2.getOrigin(); 

    listener.waitForTransform("sl_base", "sl_3", ros::Time(0), ros::Duration(1.0)); 
    listener.lookupTransform("sl_base", "sl_3", ros::Time(0), transform3); 
    tf::Vector3 t3 = transform3.getOrigin(); 

    listener.waitForTransform("sl_base", "sl_4", ros::Time(0), ros::Duration(1.0)); 
    listener.lookupTransform("sl_base", "sl_4", ros::Time(0), transform4); 
    tf::Vector3 t4 = transform4.getOrigin(); 
    t4.setZ(t4.getZ() + 0.0);
    transform4.setOrigin(t4);

    t2.setZ((t2.getZ() + t3.getZ())/2); // Obliczenie środka tf2 jako średniej położenia tf2 i tf3 
    t2.setY((t2.getY() + t3.getY())/2);
    t2.setX((t2.getX() + t3.getX())/2);
    transform2.setOrigin(t2);

    t3.setZ((t3.getZ() + t4.getZ())/2); //Obliczenie środka tf2 jako średniej położenia tf3 i tf4 
    t3.setY((t3.getY() + t4.getY())/2);
    t3.setX((t3.getX() + t4.getX())/2);
    transform3.setOrigin(t3);

    br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "sl_base", "tf_1")); 
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "sl_base", "tf_2"));
    br.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "sl_base", "tf_3"));
    br.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "sl_base", "tf_4"));

    // Człony robota będą przybliżone 4 prostopadłscianami obieky klasy fcl::box
    std::shared_ptr<fcl::Box> l1 = std::make_shared<fcl::Box>(0.1, 0.1, 0.33); // Ustawienie wymiaru boxa
    fcl::Quaternion3f l1_q(transform1.getRotation().getW(), transform1.getRotation().getX(), transform1.getRotation().getY(), transform1.getRotation().getZ()); // ustawienie rotacji
    fcl::Vec3f l1_T(transform1.getOrigin().getX(), transform1.getOrigin().getY(), transform1.getOrigin().getZ()); // ustawienie środka boxa
    fcl::Transform3f l1_R(l1_q, l1_T); // utowrzenie obiektu fcl::Transform3f
    fcl::CollisionObject* c1 = new fcl::CollisionObject(l1, l1_R); // utworzenie nowego obiektu klasy fcl::CollisionObject

    std::shared_ptr<fcl::Box> l2 = std::make_shared<fcl::Box>(0.19, 0.19, 0.72);
    fcl::Quaternion3f l2_q(transform2.getRotation().getW(), transform2.getRotation().getX(), transform2.getRotation().getY(), transform2.getRotation().getZ());
    fcl::Vec3f l2_T(transform2.getOrigin().getX(), transform2.getOrigin().getY(), transform2.getOrigin().getZ());
    fcl::Transform3f l2_R(l2_q, l2_T);
    fcl::CollisionObject* c2 = new fcl::CollisionObject(l2, l2_R);

    std::shared_ptr<fcl::Box> l3 = std::make_shared<fcl::Box>(0.16, 0.16, 0.5);
    fcl::Quaternion3f l3_q(transform3.getRotation().getW(), transform3.getRotation().getX(), transform3.getRotation().getY(), transform3.getRotation().getZ());
    fcl::Vec3f l3_T(transform3.getOrigin().getX(), transform3.getOrigin().getY(), transform3.getOrigin().getZ());
    fcl::Transform3f l3_R(l3_q, l3_T);
    fcl::CollisionObject* c3 = new fcl::CollisionObject(l3, l3_R);

    std::shared_ptr<fcl::Box> l4 = std::make_shared<fcl::Box>(0.2, 0.2, 0.2);
    fcl::Quaternion3f l4_q(transform4.getRotation().getW(), transform4.getRotation().getX(), transform4.getRotation().getY(), transform4.getRotation().getZ());
    fcl::Vec3f l4_T(transform4.getOrigin().getX(), transform4.getOrigin().getY(), transform4.getOrigin().getZ());
    fcl::Transform3f l4_R(l4_q, l4_T);
    fcl::CollisionObject* c4 = new fcl::CollisionObject(l4, l4_R);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    if (temp_cloud->points.size() == 0) // Jeśli brak człowieka w otoczeniu (chmura punktów jest pusta)
        return -1; // zwroc -1 i wyjdz z funkcji

    int ile = 0; // Zmienna zliczająca liczbę punktów w przesztrzeni robocznej robota
    for (int i = 0; i < temp_cloud->points.size(); i++) // Sprawdzenie czy punkty z chmury znajdują się w okręgu o promieniu 1m (w przybliżeniu obszar roboczy robota)
    {
        if (temp_cloud->points[i].x < workspace_size  &&  temp_cloud->points[i].y < workspace_size  && temp_cloud->points[i].z < workspace_size)
            ile ++;
    }

    if (ile  > prog1) // // Jeśli wykryto przynajmnjej 10 kolizj, w chmurze punktów występują szumy które mogłby by dać błędny wynik działania
    {
        colliosion_result = 1; // ustaw zmienna colliosion_result = 1
    }

    // Utworzenie wektora sześcianów z chmury punktów
    std::vector <fcl::CollisionObject*> boxes; 
    for (int i = 0; i < temp_cloud->points.size(); i++)
    {
        std::shared_ptr<fcl::Box> box = std::make_shared<fcl::Box>(box_size, box_size, box_size); // utowrzenie boxa o wybranym rozmiarze
        fcl::CollisionObject* col_box = new fcl::CollisionObject(box, fcl::Transform3f(fcl::Vec3f(temp_cloud->points[i].x, temp_cloud->points[i].y, temp_cloud->points[i].z))); //ustaweienie pozycji boxa
        boxes.push_back(col_box); // dodanie do vectora
    }

    fcl::CollisionRequest request; 
    fcl::CollisionResult result; // zmienna na wynik detekcji kolizj
    int ile_kolizji = 0; // zmienna zliczająca liczbe wykrytych kolizj

    for (int j = 0; j < boxes.size(); j++) // dla każdego boxa sprawdź kolizje z każdym członem
    {
        fcl::collide(c1, boxes[j], request, result);
        if (result.isCollision())       
            ile_kolizji++;

        fcl::collide(c2, boxes[j], request, result);
        if (result.isCollision())
            ile_kolizji++;

        fcl::collide(c3, boxes[j], request, result);
        if (result.isCollision())
            ile_kolizji++;

        fcl::collide(c4, boxes[j], request, result);
        if (result.isCollision())
            ile_kolizji++;
    }

    if (ile_kolizji > prog2) // Jeśli wykryto przynajmnjej 3 kolizj, w chmurze punktów występują szumy które mogłby by dać błędny wynik działania
        colliosion_result = 2; // ustaw zmienna colliosion_result = 2

    for (int j = 0; j < boxes.size(); j++)
    {
       delete boxes[j];
    }

    return colliosion_result;
}

void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    time_scalling.data = 1.0;
    int collision_result = check_collision(msg, 1.1, 0.01, 15, 3);

    switch (collision_result) // Komunikaty i ustalenie ograczniczenia prędkości w zależności od wyniku detekcji kolzji
    {
    case -1:
        time_scalling.data = previous_value;
        ROS_INFO("Nie wykryto czlowieka w otoczeniu");
    break;

    case 0:
        time_scalling.data = 1.0;
        ROS_INFO("Brak czlowieka w obszarze roboczym");
    break;

    case 1:
        time_scalling.data = 0.5;
        ROS_WARN("Czlowiek w obszarze roboczym");
    break;

    case 2:
        time_scalling.data = 0.1;
        ROS_ERROR("Czlowiek w poblizu robota, mozliwa kolizja!!!");
    break;
    }

    // Zliczanie ile razy z rzędu nie wykryto człowieka w otoczeniu
    if(collision_result == -1){
        no_human_counter++;
    }
    else {
        no_human_counter = 0;
    }
    
    if(no_human_counter > 20) // Jeśli człowieka nie wykryto 30 razy z rzędu 
        time_scalling.data = 1.0; // ustaw skalowanie prędkości na 1

    previous_value = time_scalling.data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "colision_detection");
    ros::NodeHandle n;

    ros::Subscriber point_cloud_sub = n.subscribe("/kinect2/hd/points", 1, point_cloud_callback);
    ros::Publisher vel_pub = n.advertise<std_msgs::Float32>("/es_trajectory/time_scaling", 1000);
    ros::Publisher col_flag_pub = n.advertise<std_msgs::Bool>("/colision_flag", 1000);
    
    ros::Rate loop_rate(30);

    while(ros::ok)
    {
    loop_rate.sleep();
    ros::spinOnce();
    vel_pub.publish(time_scalling);
    }
    return 0;
}