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
#include <opencv2/imgproc/imgproc.hpp>

using namespace grid_map;
using namespace ros;


image_transport::Publisher pub;
static const std::string OPENCV_WINDOW = "Image window";

void cb(const grid_map_msgs::GridMap& msg){

  grid_map::GridMap map;


  // cv_bridge::CvImage image;
  sensor_msgs::Image image;
  cv_bridge::CvImage cvImage;
  //this seems inefficient but the gridmap to cvimage function does the same thing under the hood
  grid_map::GridMapRosConverter::fromMessage(msg, map, {"color"});
  grid_map::GridMapRosConverter::toCvImage(map,"color", sensor_msgs::image_encodings::RGBA16, cvImage);

  ROS_INFO("Received map with size %f x %f m (%i x %i cells).",
  map.getLength().x(), map.getLength().y(),
  map.getSize()(0), map.getSize()(1));

  // cv::Mat originalImage;
  // const std::string layer = "elevation";
  // GridMapCvConverter::toImage(map, layer, sensor_msgs::image_encodings::MONO8, image);

 

  cvImage.toImageMsg(image);
  cv_bridge::CvImagePtr cv_ptr;
 try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGBA16);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
      cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

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