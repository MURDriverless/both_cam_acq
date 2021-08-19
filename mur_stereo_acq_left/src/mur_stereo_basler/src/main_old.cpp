/*
Acquires images from the left camera and publishes them on a ROS Topic named /imagePublisher/left_image

lightSetting 
1 - off
2 - daylight 5000K
3 - daylight 6500K
4 - Fluorescent 4000K
5 - Tungsten 2800K

exposureTimeSetting 
1 - defaultExposureTime == 5000us
                

By: Kelvin Liao, Spatial and Perception Engineer at MUR 2020
Date: 17/08/2021 

Adapted from: Andrew Huang, Spatial and Perception Engineer at MUR 2020

*/
#include <ros/duration.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>
#include <iostream>
#include <ros/ros.h> // Must include for all ROS C++
#include "sensor_msgs/Image.h" 
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "Detectors.hpp"
#include "ClassicalStereo.hpp"
#include "PreviewArgs.hpp"
#include "GeniWrap.hpp"

#define PREVIEW

#ifndef SRC_ROOT_PATH
#define SRC_ROOT_PATH "./"
#endif

const std::string CAMARA_NAME_L = "CameraLeft (40068492)";

unsigned int grabCount =6000;

static const std::string OPENCV_WINDOW = "Left Image Window";

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter():it_(nh_) {
    image_sub_ = it_.subscribe("/imagePublisher/left_image", 1,
    &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/imagePublisher/left_image", 1);

  }

  ~ImageConverter() {

  }

  // this imageCB function is used to convert ROS messages to OpenCV messages 
  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cv::Mat dst;
    // cv::cvtColor(cv_ptr->image, dst, cv::COLOR_BGR2RGB); 
    
    // char image_name[256];
    // std::sprintf(image_name,"CameraLeft%04d_L.png", std::stoi(cv_ptr->header.frame_id));
    // cv::imwrite(image_name, dst);
    // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, dst);
    // cv::waitKey(100);

    // std::cout << "The frame ID is: " << cv_ptr->header.frame_id << std::endl; 
    // std::cout << "The frame time is: " << cv_ptr->header.stamp << std::endl;
  }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "Left_Image_Acquisition");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ros::Publisher left_imagePublisher = nh.advertise<sensor_msgs::Image>("/imagePublisher/left_image", 1);

    ImageConverter ic; 

    // Prep real cameras
    std::unique_ptr<IGeniCam> camera1;

    // run the cameras 
    camera1.reset(IGeniCam::create(GeniImpl::Pylon_i));
    camera1->initializeLibrary();
    
    int exitCode = 0;

    int imageCount = 0;

    int lightSetting = 1;
    int exposureTimeSetting = 1;
    int frameRateSetting = 1;

    if (argc > 1) {  
      lightSetting = std::stoi(argv[1]);
      exposureTimeSetting = std::stoi(argv[2]);
      frameRateSetting = std::stoi(argv[3]);
      std::cout << "reading successful" << std::endl;
    }

    try {
        // std::cout << "try block" << std::endl;

        camera1->setup(CAMARA_NAME_L, lightSetting, exposureTimeSetting, frameRateSetting);

        camera1->startGrabbing(grabCount);
        
        ros::Time start_time = ros::Time::now();
        auto first_camera1 = std::chrono::high_resolution_clock::now();
        auto camera1_prev = first_camera1;
        double frameTime1;
        double frameTime2;
        auto camera1_done = first_camera1;

        while (imageCount < grabCount && camera1->isGrabbing() && ros::ok()) {
          
          // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
          int height1;
          int width1;
          uint8_t* buffer1;

          // std::cout << "while loop" << std::endl;

          // if (camera1->isReady()) {
          //   std::cout << "camera1 is ready" << std::endl;
          //   camera1->softwareTrigger();
          // }

          // camera1->softwareTrigger();

          bool ret1 = camera1->retreiveResult(height1, width1, buffer1);
          ros::Time camera1_time = ros::Time::now();
          

          sensor_msgs::Image left_img_msg;

          camera1_done = std::chrono::high_resolution_clock::now();
          
          frameTime1 = std::chrono::duration_cast<std::chrono::milliseconds>(camera1_done - first_camera1).count();
          frameTime2 = std::chrono::duration_cast<std::chrono::milliseconds>(camera1_done - camera1_prev).count();

          std::cout << "Camera Left Frame Rate is: " << camera1->getFrameRate();
          std::cout << ", Exposure Time is: " << camera1->getExposureTime();
          std::cout << ", Temperature reading is: " << camera1->getTemperatureReading(); 
          std::cout << ", AbsTime is: " << frameTime1 << " ms";
          std::cout << ", RelativeTime is: " << frameTime2 << std::endl;
          
          if (ret1) {
            cv_bridge::CvImage myCvImage; 

            // std::cout << "ret1 " << std::endl;

            // put the image into a CvImage object 
            myCvImage.image = cv::Mat(height1, width1, CV_8UC3, buffer1);
            myCvImage.encoding = "bgr8";

            // ros::Time diff = camera1_time - start_time;
            myCvImage.header.stamp = camera1_time;
            myCvImage.header.frame_id = std::to_string(imageCount); 

            // convert the CvImage object into a ROS image 
            myCvImage.toImageMsg(left_img_msg);

            cv::Mat dst;
            cv::cvtColor(myCvImage.image, dst, cv::COLOR_BGR2RGB); 
            cv::imshow("left image", dst);
            cv::waitKey(100);

            // char image_name[256];
            // std::sprintf(image_name,"CameraLeft%04d_L.png", imageCount);
            // cv::imwrite(image_name, dst);

          }
       
          camera1_prev = camera1_done;
          imageCount = imageCount + 1;
          left_imagePublisher.publish(left_img_msg);
          ros::spinOnce();

          if (camera1->getTemperatureReading() > 75) {
            std::cout << "CRITICAL temperature state, closing camera" << std::endl;
            break; 
         }

      }
    }
    catch (const std::exception &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
        << e.what() << std::endl;
        exitCode = 1;
    }

    std::cout << "ending program" << std::endl;
    camera1->finalizeLibrary();
    delete camera1.release();
    return exitCode;
}