/*
https://github.com/ANYbotics/grid_map/blob/master/grid_map_demos/src/opencv_demo_node.cpp
*/
#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_core/gtest_eigen.hpp>
 #include <image_transport/image_transport.h>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <sensor_msgs/image_encodings.h>

using namespace grid_map;
using namespace ros;


image_transport::Publisher pub;
// int main(int argc, char** argv)
// {


//   // Create grid map.
//   GridMap map({"original", "elevation"});
//   map.setFrameId("map");
//   map.setGeometry(Length(1.2, 2.0), 0.01);
//   ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
//     map.getLength().x(), map.getLength().y(),
//     map.getSize()(0), map.getSize()(1));

  
//   // Convert to CV image.
//   cv::Mat originalImage;
//   if (useTransparency) {
//     // Note: The template parameters have to be set based on your encoding
//     // of the image. For 8-bit images use `unsigned char`.
//     GridMapCvConverter::toImage<unsigned short, 4>(map, "original", CV_16UC4, 0.0, 0.3, originalImage);
//   } else {
//     GridMapCvConverter::toImage<unsigned short, 1>(map, "original", CV_16UC1, 0.0, 0.3, originalImage);
//   }

//   // Create OpenCV window.
//   cv::namedWindow("OpenCV Demo");

//   // Work with copy of image in a loop.
//   while (nodeHandle.ok()) {

//     // Initialize.
//     ros::Time time = ros::Time::now();
//     map.setTimestamp(time.toNSec());
//     cv::Mat modifiedImage;
//     int blurRadius = 200 - abs((int)(200.0 * sin(time.toSec())));
//     blurRadius = blurRadius - (blurRadius % 2) + 1;

//     // Apply Gaussian blur.
//     cv::GaussianBlur(originalImage, modifiedImage, cv::Size(blurRadius, blurRadius), 0.0, 0.0);

//     // Visualize as image.
//     cv::imshow("OpenCV Demo", modifiedImage);
//     cv::waitKey(40);

 
//   }

//   return 0;
// }

void cb(const grid_map_msgs::GridMap& msg){

    grid_map::GridMap map;


    // cv_bridge::CvImage image;
    sensor_msgs::Image image;
 cv_bridge::CvImage cvImage;
    grid_map::GridMapRosConverter::fromMessage(msg, map, {"color"});


    grid_map::GridMapRosConverter::toCvImage(map,"color", sensor_msgs::image_encodings::BGR8, cvImage);

    ROS_INFO("Received map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
      const float minValue = -1.0;
  const float maxValue = 1.0;

    // cv::Mat originalImage;
    // const std::string layer = "elevation";
    // GridMapCvConverter::toImage(map, layer, sensor_msgs::image_encodings::MONO8, image);
  
 

  cvImage.toImageMsg(image);
  pub.publish(image);

    // image_pub_.publish(image.toImageMsg());

}

static const std::string IMAGE_TOPIC = "/elevation_mapping/elevation_map_raw";
static const std::string PUBLISH_TOPIC = "/image_msg";

int main (int argc, char** argv)
{
  // Initialize the ROS Node "roscpp_pcl_example"
  ros::init (argc, argv, "gm2cv_example");
  ros::NodeHandle nh;

 
  // Print "Hello" message with node name to the terminal and ROS log file
  ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

  // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
  ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cb);

  image_transport::ImageTransport it(nh);
  pub = it.advertise(PUBLISH_TOPIC, 1);




  // Spin
  ros::spin();

  // Success
  return 0;
}