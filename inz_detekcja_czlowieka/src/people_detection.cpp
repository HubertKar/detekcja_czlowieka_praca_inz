#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/exceptions.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/people/people_detector.h>
#include <pcl/gpu/people/colormap.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/io/oni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/filesystem.hpp>

#include <iostream>

////////////////////////////////// ROS ///////////////
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include "ros/ros.h"
#include <ros/package.h>
 
using namespace std::literals::chrono_literals;
namespace pc = pcl::console;
using namespace pcl::visualization;
using namespace pcl::gpu;
using namespace pcl;

class PeoplePCDApp
{
  public:
    typedef pcl::gpu::people::PeopleDetector PeopleDetector;

    enum { COLS = 640, ROWS = 480 };

    PeoplePCDApp (pcl::Grabber& capture) : capture_(capture), exit_(false), time_ms_(0), cloud_cb_(true), counter_(0), final_view_("Final labeling"), depth_view_("Depth")
    {
      final_view_.setSize (COLS, ROWS);
      depth_view_.setSize (COLS, ROWS);

      final_view_.setPosition (0, 0);
      depth_view_.setPosition (650, 0);

      cmap_device_.create(ROWS, COLS);
      cmap_host_.resize(COLS * ROWS);
      depth_device_.create(ROWS, COLS);
      image_device_.create(ROWS, COLS);

      depth_host_.resize(COLS * ROWS);

      rgba_host_.resize(COLS * ROWS);
      rgb_host_.resize(COLS * ROWS * 3);

      people::uploadColorMap(color_map_);
    }

    void
    visualizeAndWrite(bool write = false)
    {
      const PeopleDetector::Labels& labels = people_detector_.rdf_detector_->getLabels();
      people::colorizeLabels(color_map_, labels, cmap_device_);

      int c;
      cmap_host_.width = cmap_device_.cols();
      cmap_host_.height = cmap_device_.rows();
      cmap_host_.resize(cmap_host_.width * cmap_host_.height);
      cmap_device_.download(cmap_host_.points, c);

      final_view_.showRGBImage<pcl::RGB>(cmap_host_);
      final_view_.spinOnce(1, true);

      if (cloud_cb_)
      {
        depth_host_.width = people_detector_.depth_device2_.cols();
        depth_host_.height = people_detector_.depth_device2_.rows();
        depth_host_.resize(depth_host_.width * depth_host_.height);
        people_detector_.depth_device2_.download(depth_host_.points, c);
      }

      depth_view_.showShortImage(&depth_host_[0], depth_host_.width, depth_host_.height, 0, 5000, true);
      depth_view_.spinOnce(1, true);
    }

    void source_cb1(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
    {
      {
        std::lock_guard<std::mutex> lock(data_ready_mutex_);
        if (exit_)
          return;
        pcl::copyPointCloud(*cloud, cloud_host_);
      }
      data_ready_cond_.notify_one();
    }

    // nie używane
    void source_cb2(const pcl::io::openni2::Image::Ptr& image_wrapper, const pcl::io::openni2::DepthImage::Ptr& depth_wrapper, float)
    {
    }


    void
    startMainLoop (int argc, char** argv)
    {
      cloud_cb_ = true; // wykorzystaj chmure punktów zamiast obrazu z kamery RGB i IR 

      ///////////////////ROS////////////////////////
      ros::init(argc, argv, "people_detector");

      sensor_msgs::PointCloud2 point_cloud_2_;
      sensor_msgs::PointCloud2 tmp_point_cloud_2_;

      ros::NodeHandle nh_;
      ros::Publisher point_cloud_pub_;
      ros::Publisher image_pub_;
      ros::Publisher image_people_pub_;
      tf::TransformListener listener;

      tf::StampedTransform  point_cloud_trnasform;
      point_cloud_trnasform.setOrigin(tf::Vector3(0,0,0));
      point_cloud_trnasform.setRotation(tf::Quaternion(0,0,0,1));

      point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
      image_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/RGB_image", 20);
      image_people_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/RGB_people_image", 1);
      ///////////////////ROS///////////////////////

      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBA> filter;

      // Wskażniki do obrazów 
      typedef pcl::io::openni2::DepthImage::Ptr DepthImagePtr;
      typedef pcl::io::openni2::Image::Ptr ImagePtr;

      // callback (uzywany jest func1)
      std::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> func1 = [this] (const PointCloud<PointXYZRGBA>::ConstPtr& cloud) { source_cb1 (cloud); };
      std::function<void (const ImagePtr&, const DepthImagePtr&, float)> func2 = [this] (const ImagePtr& img, const DepthImagePtr& depth, float constant)
      {
        source_cb2 (img, depth, constant);
      };
      boost::signals2::connection c = cloud_cb_ ? capture_.registerCallback (func1) : capture_.registerCallback (func2);

      {
        std::unique_lock<std::mutex> lock(data_ready_mutex_);

        try
        {
          capture_.start ();
          while (!exit_ && !final_view_.wasStopped() && ros::ok)
          {
            // czekanie na dane
            bool has_data = (data_ready_cond_.wait_for(lock, 100ms) == std::cv_status::no_timeout);
            if(has_data)
            {
              //SampledScopeTime fps(time_ms_);
                process_return_ = people_detector_.process(cloud_host_.makeShared()); // przetwarzanie chmury punktów
              ++counter_;
            }

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGBA>); //deklaracja chmury punktow po filtracji (tylko punkty na których jest człowiek)

            if(has_data && (process_return_ == 2))  // Jeśli są dane i proces się nie zakończył
            {
              visualizeAndWrite(); // Wizualizacja danych 

              for (std::size_t i = 0; i < depth_host_.size(); i++ ) // Dodanie do chmury punktow tylko punktow na których znajduje się człowiek
              {
                  if(depth_host_[i] < 10000) // Jeśli wartość jest mnjiesza od jakiegoś progu, pole na którym nie ma człowieka przyjmuje wartoś maksymalną
                  {
                    pcl::_PointXYZRGBA point = cloud_host_[i]; // Odwrócenie wartości x punktu
                    point.x = -point.x;
                    filteredCloud->push_back(point);
                  }  
              }

              // Filtrowanie chmury punktów statystyczne
              filter.setInputCloud(filteredCloud);
              filter.setMeanK(50);
              filter.setStddevMulThresh(1.0);
	            filter.filter(*filteredCloud);
              
              try
              {
              pcl::toROSMsg(cloud_host_, RGB_image); 
              image_pub_.publish(RGB_image);

              pcl::toROSMsg(cmap_host_, RGB_image_people);
              image_people_pub_.publish(RGB_image_people); //konwersja z chmury punktów na sensor_msgs::Image
              }
              catch (std::runtime_error e)
              {
                ROS_ERROR_STREAM("Error in converting cloud to image message: " << e.what());
              }

            }

              try{
              listener.lookupTransform("sl_base", "kinect_frame", ros::Time(0), point_cloud_trnasform);
              } catch (tf::TransformException ex)
              {}           

              pcl::toROSMsg(*filteredCloud, tmp_point_cloud_2_);
              pcl_ros::transformPointCloud("sl_base", point_cloud_trnasform, tmp_point_cloud_2_, point_cloud_2_); // tranformacja chmury punktów 

              point_cloud_2_.header.frame_id = "sl_base";  // frame
              point_cloud_2_.header.stamp = ros::Time::now(); // czas
              point_cloud_pub_.publish(point_cloud_2_); // publikowanie wiadomości
          }

          final_view_.spinOnce (3);
        }
        catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
        catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

        capture_.stop ();
      }
      c.disconnect();
    }

    std::mutex data_ready_mutex_;
    std::condition_variable data_ready_cond_;

    pcl::Grabber& capture_;

    bool cloud_cb_;
    bool exit_;
    int time_ms_;
    int counter_;
    int process_return_;
    PeopleDetector people_detector_;
    PeopleDetector::Image cmap_device_;
    pcl::PointCloud<pcl::RGB> cmap_host_;

    PeopleDetector::Depth depth_device_;
    PeopleDetector::Image image_device_;

    pcl::PointCloud<unsigned short> depth_host_;
    pcl::PointCloud<pcl::RGB> rgba_host_;
    std::vector<unsigned char> rgb_host_;

    PointCloud<PointXYZRGBA> cloud_host_;

    ImageViewer final_view_;
    ImageViewer depth_view_;

    DeviceArray<pcl::RGB> color_map_;

    ////ROS /////
    sensor_msgs::Image RGB_image;
    sensor_msgs::Image RGB_image_people;
};


int main(int argc, char** argv)
{
  // selecting GPU and prining info
  int device = 0;
  pc::parse_argument (argc, argv, "-gpu", device);
  pcl::gpu::setDevice (device);
  pcl::gpu::printShortCudaDeviceInfo (device);

  // selecting data source 
  // wersja działająca dla kinecta V1 niżej jest wersja dla kinecta v2
  //pcl::Grabber::Ptr capture (new pcl::OpenNIGrabber());
  //pcl::shared_ptr<pcl::Grabber> capture;
  //capture.reset( new pcl::OpenNIGrabber() ); 

  // Openni2 grabber, służący do pobierania danych z kinecta
  pcl::io::OpenNI2Grabber *OpenNI2Grabber = new pcl::io::OpenNI2Grabber();

  // Podanie wartości do falibracji kamer fx, fy, cx, cy

  //OpenNI2Grabber->setRGBFocalLength(1081.3720703125, 1081.3720703125);
  //OpenNI2Grabber->setRGBCameraIntrinsics(1081.3720703125, 1081.3720703125, 959.5, 539.5); 

  //OpenNI2Grabber->setDepthFocalLength(365.92559814453125, 365.92559814453125);
  //OpenNI2Grabber->setDepthCameraIntrinsics(365.92559814453125, 365.92559814453125, 257.74468994140625, 214.39199829101562);
  
  pcl::Grabber *capture = OpenNI2Grabber;

  //selecting tree files Wczytanie plików
  std::vector<std::string> tree_files;
  std::string const package_path = ros::package::getPath("inz_detekcja_czlowieka");

  tree_files.push_back(package_path + "/Data/forest1/tree_20.txt");
  tree_files.push_back(package_path + "/Data/forest2/tree_20.txt");
  tree_files.push_back(package_path + "/Data/forest3/tree_20.txt");

  pc::parse_argument (argc, argv, "-tree0", tree_files[0]);
  pc::parse_argument (argc, argv, "-tree1", tree_files[1]);
  pc::parse_argument (argc, argv, "-tree2", tree_files[2]);

  int num_trees = (int)tree_files.size();
  pc::parse_argument (argc, argv, "-numTrees", num_trees);

  tree_files.resize(num_trees);
  if (num_trees == 0 || num_trees > 3)
    return std::cout << "Invalid number of trees" << std::endl, -1;

  try
  {
    // loading trees nie wiem jak to narzie przetłumaczyć
    typedef pcl::gpu::people::RDFBodyPartsDetector RDFBodyPartsDetector;
    RDFBodyPartsDetector::Ptr rdf(new RDFBodyPartsDetector(tree_files));
    PCL_INFO("Loaded files into rdf");

    // Tworzenie obiektu klast PeoplePCDApp
    PeoplePCDApp app(*capture);
    app.people_detector_.rdf_detector_ = rdf;

    // główna pętla programu
    app.startMainLoop (argc, argv);
  }
  catch (const pcl::PCLException& e) { std::cout << "PCLException: " << e.detailedMessage() << std::endl; }  
  catch (const std::runtime_error& e) { std::cout << e.what() << std::endl; }
  catch (const std::bad_alloc& /*e*/) { std::cout << "Bad alloc" << std::endl; }
  catch (const std::exception& /*e*/) { std::cout << "Exception" << std::endl; }

  return 0;
}