/*
Acquires images from the left camera and publishes them on a ROS Topic named /imagePublisher/left_image

lightSetting 
1 - off
2 - daylight 5000K
3 - daylight 6500K

exposureTimeSetting 
1 - defaultExposureTime == 5000us
                

By: Kelvin Liao, Spatial and Perception Engineer at MUR 2020
Date: 17/08/2021 

Adapted from: Andrew Huang, Spatial and Perception Engineer at MUR 2020

*/

// ********************************** LIBRARIES *********************************************
#include <iostream>
#include <unistd.h>
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

// *************************** CAMERA SERIAL NUMBER ************************************************
const std::string CAMARA_NAME_L = "CameraLeft (40068492)";
const std::string CAMARA_NAME_R = "CameraRight (40061679)";

unsigned int grabCount = 6000;

// ************* IMAGE CONVERTER - FOR PUBLISHING AND SUBSCRIBING TO ROS MESSAGES *******************
class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_left;
  image_transport::Publisher image_pub_left;

  image_transport::Subscriber image_sub_right;
  image_transport::Publisher image_pub_right;

public:
  ImageConverter():it_(nh_) {
    image_sub_left = it_.subscribe("/imagePublisher/left_image", 1,
    &ImageConverter::imageCb, this);
    image_pub_left = it_.advertise("/imagePublisher/left_image", 1);

    image_sub_right = it_.subscribe("/imagePublisher/right_image", 1,
    &ImageConverter::imageCb, this);
    image_pub_right = it_.advertise("/imagePublisher/right_image", 1);

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

// ***********************************************************************************************
int main(int argc, char** argv) {

    ros::init(argc, argv, "Both_Image_Acquisition");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ros::Publisher left_imagePublisher = nh.advertise<sensor_msgs::Image>("/imagePublisher/left_image", 1);
    ros::Publisher right_imagePublisher = nh.advertise<sensor_msgs::Image>("/imagePublisher/right_image", 1);
    ImageConverter ic; 

    int lightSetting = 1;
    int exposureTimeSetting = 1;
    int frameRateSetting = 1;
    int exitCode = 0;
    int capturePhoto = 0;
    
    if (argc > 1) {  
      lightSetting = std::stoi(argv[1]);
      exposureTimeSetting = std::stoi(argv[2]);
      frameRateSetting = std::stoi(argv[3]);
      capturePhoto = std::stoi(argv[4]);
      std::cout << "reading successful" << std::endl;
    }

    // Prep real cameras
    std::unique_ptr<IGeniCam> camera1;
    std::unique_ptr<IGeniCam> camera2;

    // run the cameras 
    camera1.reset(IGeniCam::create(GeniImpl::Pylon_i));
    camera2.reset(IGeniCam::create(GeniImpl::Pylon_i));

    camera1->initializeLibrary();
    camera2->initializeLibrary();
    
    try {
        camera1->setup(CAMARA_NAME_L, lightSetting, exposureTimeSetting, frameRateSetting);
        camera2->setup(CAMARA_NAME_R, lightSetting, exposureTimeSetting, frameRateSetting);

        ros::Time start_time = ros::Time::now();

        int imageCount = 0;

        camera1->startGrabbing(grabCount);
        camera2->startGrabbing(grabCount);


        while(imageCount < grabCount && ros::ok()) {
          
            for (int i = 0; i < 3; i++) {
                if (camera1->isReady() & camera2->isReady()) {
                    std::cout << "both cameras are ready: " << imageCount << std::endl;
                    camera1->softwareTrigger();
                    camera2->softwareTrigger();   
                    break;
                }
                else {
                  // std::cout << "not ready" << std::endl;
                }
            } 

            sleep(0.5);

            bool flag = true;

            int height1;
            int width1;
            uint8_t* buffer1;

            int height2;
            int width2;
            uint8_t* buffer2;
            
            bool ret1 = camera1->retrieveResult(height1, width1, buffer1);
            bool ret2 = camera2->retrieveResult(height2, width2, buffer2);

            std::cout << "Camera Left Frame Rate is: " << camera1->getFrameRate();
            std::cout << ", Exposure Time is: " << camera1->getExposureTime();
            std::cout << ", Temperature reading is: " << camera1->getTemperatureReading() << std::endl; 

            ros::Time camera1_time = ros::Time::now();
            
            sensor_msgs::Image left_img_msg;
            sensor_msgs::Image right_img_msg;

            cv::Mat dst2;

            std::cout << "retrieved images" << std::endl;

            if (ret1 && ret2) {
                cv_bridge::CvImage myCvImage_left;
                cv_bridge::CvImage myCvImage_right;  

                myCvImage_left.image = cv::Mat(height1, width1, CV_8UC3, buffer1);
                myCvImage_left.encoding = "bgr8";

                // ros::Time diff = camera1_time - start_time;
                myCvImage_left.header.stamp = camera1_time;
                myCvImage_left.header.frame_id = std::to_string(imageCount); 

                // convert the CvImage object into a ROS image 
                myCvImage_left.toImageMsg(left_img_msg);

                myCvImage_right.image = cv::Mat(height1, width1, CV_8UC3, buffer2);
                myCvImage_right.encoding = "bgr8";

                // ros::Time diff = camera1_time - start_time;
                myCvImage_right.header.stamp = camera1_time;
                myCvImage_right.header.frame_id = std::to_string(imageCount); 

                // convert the CvImage object into a ROS image 
                myCvImage_right.toImageMsg(right_img_msg);                    

                if (capturePhoto == 1) {
                    // cv::Mat inMat = cv::Mat(height1, width1, CV_8UC3, buffer1);
                    cv::Mat dst;
                    cv::cvtColor(myCvImage_left.image, dst, cv::COLOR_BGR2RGB); 
                    // cv::imshow("Left camera", dst);
                    // cv::waitKey(100);

                    char image_name[256];
                    std::sprintf(image_name,"CameraLeft%04d_L.png", imageCount);
                    cv::imwrite(image_name, dst);

                    // cv::Mat inMat2 = cv::Mat(height2, width2, CV_8UC3, buffer2);
                    cv::Mat dst2;
                    cv::cvtColor(myCvImage_right.image, dst2, cv::COLOR_BGR2RGB); 
                    // cv::imshow("Right camera", dst2);
                    // cv::waitKey(100);

                    char image_name2[256];
                    std::sprintf(image_name2,"CameraRight%04d_R.png", imageCount);
                    cv::imwrite(image_name2, dst2);

                }
            }
            imageCount++;

            left_imagePublisher.publish(left_img_msg);
            right_imagePublisher.publish(right_img_msg);

            if (camera1->getTemperatureReading() > 75) {
                std::cout << "CRITICAL temperature state, closing camera" << std::endl;
                break; 
            }

            ros::spinOnce();
        }

        camera1->stopGrabbing();
        camera2->stopGrabbing();
    }   
    catch (const std::exception &e)
    {
        std::cout << "entered catch block" << std::endl;
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
        << e.what() << std::endl;
        exitCode = 1;
    }

    std::cout << "ending program" << std::endl;
    camera1->finalizeLibrary();
    camera2->finalizeLibrary();

    delete camera1.release();
    delete camera2.release();

    return exitCode;
}