/*

23.06.2021
What does this code do?

It uses the cameras to capture images 


*/

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

const std::string CAMARA_NAME_L = "CameraLeft (40068492)";
const std::string CAMARA_NAME_R = "CameraRight (40061679)";

unsigned int grabCount = 6000;

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

    if (argc > 1) {  
      lightSetting = std::stoi(argv[1]);
      exposureTimeSetting = std::stoi(argv[2]);
      frameRateSetting = std::stoi(argv[3]);
      std::cout << "reading successful" << std::endl;
    }

    // Prep real cameras
    std::unique_ptr<IGeniCam> camera1;
    std::unique_ptr<IGeniCam> camera2;

    // std::cout << "before geni code" << std::endl;

    // run the cameras 
    camera1.reset(IGeniCam::create(GeniImpl::Pylon_i));
    camera2.reset(IGeniCam::create(GeniImpl::Pylon_i));
    
    // std::cout << "after geni cam code" << std::endl;

    camera1->initializeLibrary();
    camera2->initializeLibrary();
    
    int exitCode = 0;

    // std::cout << "initialising library" << std::endl;

// leftcamera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
// rightcamera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

// for i in range(3):
//     if leftcamera.WaitForFrameTriggerReady(200,pylon.TimeoutHandling_ThrowException) & rightcamera.WaitForFrameTriggerReady(200, pylon.TimeoutHandling_ThrowException):
//         leftcamera.ExecuteSoftwareTrigger()
//         rightcamera.ExecuteSoftwareTrigger()

// time.sleep(0.5)

// if leftcamera.GetGrabResultWaitObject().Wait(0):
//     print("Grab results wait in the left queue.")
// if rightcamera.GetGrabResultWaitObject().Wait(0):
//     print("Grab results wait in the right queue.")

// leftresult = leftcamera.RetrieveResult(0,pylon.TimeoutHandling_Return)
// rightresult = rightcamera.RetrieveResult(0,pylon.TimeoutHandling_Return)

// timestr = time.strftime("%Y%m%d-%H%M%S")
// limg = pylon.PylonImage()
// limg.AttachGrabResultBuffer(leftresult)
// limg.Save(pylon.ImageFileFormat_Png,"Caps/left"+timestr+".png")
// rimg = pylon.PylonImage()
// rimg.AttachGrabResultBuffer(rightresult)
// rimg.Save(pylon.ImageFileFormat_Png,"Caps/right"+timestr+".png")

// leftcamera.StopGrabbing()
// rightcamera.StopGrabbing()

    try {
        camera1->setup(CAMARA_NAME_L, lightSetting, exposureTimeSetting, frameRateSetting);
        camera2->setup(CAMARA_NAME_R, lightSetting, exposureTimeSetting, frameRateSetting);

        ros::Time start_time = ros::Time::now();

        std::cout << "camera setup complete" << std::endl;

        int imageCount = 0;

        camera1->startGrabbing(grabCount);
        camera2->startGrabbing(grabCount);


        while(imageCount < grabCount && ros::ok()) {
            
            // std::cout << "round: " << imageCount << std::endl;


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

            // std::cout << "triggered via software" << std::endl;

            sleep(0.5);

            //
            // if leftcamera.GetGrabResultWaitObject().Wait(0):
            //     print("Grab results wait in the left queue.")
            // if rightcamera.GetGrabResultWaitObject().Wait(0):
            //     print("Grab results wait in the right queue.")
            
            bool flag = true;

            int height1;
            int width1;
            uint8_t* buffer1;

            int height2;
            int width2;
            uint8_t* buffer2;

            // while camera.NumReadyBuffers.GetValue() > 0:
            //   camera.RetrieveResult(5000, pylon.TimeoutHandling_Return)

            bool ret1 = camera1->retrieveResult(height1, width1, buffer1);
            bool ret2 = camera2->retrieveResult(height2, width2, buffer2);

            // std::cout << "retrieved images" << std::endl;

            ros::Time camera1_time = ros::Time::now();
            
            sensor_msgs::Image left_img_msg;
            sensor_msgs::Image right_img_msg;

            cv::Mat dst2;

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

                // cv::Mat inMat = cv::Mat(height1, width1, CV_8UC3, buffer1);
                cv::Mat dst;
                cv::cvtColor(myCvImage_left.image, dst, cv::COLOR_BGR2RGB); 
                cv::imshow("Left camera", dst);
                cv::waitKey(100);

                // char image_name[256];
                // std::sprintf(image_name,"LeftLeft%04d_L.png", imageCount);
                // cv::imwrite(image_name, dst);

                // cv::Mat inMat2 = cv::Mat(height2, width2, CV_8UC3, buffer2);
                cv::Mat dst2;
                cv::cvtColor(myCvImage_right.image, dst2, cv::COLOR_BGR2RGB); 
                cv::imshow("Right camera", dst2);
                cv::waitKey(100);

                // char image_name2[256];
                // std::sprintf(image_name2,"RightRight%04d_R.png", imageCount);
                // cv::imwrite(image_name2, dst2);
            }
            imageCount++;

            // std::cout << "before stop grabbing" << std::endl;

            // camera1->stopGrabbing();
            // camera2->stopGrabbing();

            // std::cout << "after stop grabbing" << std::endl;

            left_imagePublisher.publish(left_img_msg);
            right_imagePublisher.publish(right_img_msg);
            
            // std::cout << "published via ros" << std::endl;

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