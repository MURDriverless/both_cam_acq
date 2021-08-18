/*

23.06.2021
What does this code do?

It uses the cameras to capture images 


*/

#include <iostream>

// #include "ros/ros.h" // Must include for all ROS C++
// #include "sensor_msgs/Image.h" 
// #include <sstream>

#include <opencv2/core.hpp>

#include <unistd.h>

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

unsigned int grabCount = 20;

// // camera class 
// class StereoImageAcq{
//     //private:
//     // Variables for Current Pose
//     int height;
//     int width;
//     std::string encoding;
//     uint8_t is_bigendian;
//     uint32_t step;
//     uint8[] left_image;
//     uint8[] right_image;


//     // ROS
//     ros::NodeHandle n; // Create its specific node handler
//     ros::Publisher left_imagePublisher;
//     ros::Publisher right_imagePublisher; 
//     // ros::Subscriber poseSubscriber;


//     public:
//     // Constructor
//     StereoImageAcq() {
//         this->left_imagePublisher = n.advertise<sensor_msgs::Image>("/imagePublisher/left_image", 10);
//         this->right_imagePublisher = n.advertise<sensor_msgs::Image>("/imagePublisher/right_image", 10);
//         // this->poseSubscriber = n.subscribe("/turtle1/pose", 10, &Turtle::updatePose, this);
//         this->height = 1200; 
//         this->width = 1920; 
//     }

//     void publish_left_image() {
//         sensor_msgs::Image left_img_msg;
//         left_img_msg.data = this->left_image;

//         this->left_imagePublisher.publish(left_img_msg);
//     }

//     void publish_right_image() {
//         sensor_msgs::Image right_img_msg;
//         right_img_msg.data = this->right_image;

//         this->right_imagePublisher.publish(right_img_msg);
//     }
// };

int main(int argc, char** argv) {

    // // Initialize ROS node
    // ros::init(argc, argv, "Stereo_Images_Acquisition");

    // //Initialize Turtle Object
    // StereoImageAcq stereo_image_acq = StereoImageAcq();

    // bounding box preview
    #ifdef PREVIEW
    cv::Mat rFrameBBox;
    cv::Mat lFrameBBox;
    cv::Mat matchesPreview;
    PreviewArgs previewArgs(lFrameBBox, rFrameBBox, matchesPreview);
    #else
    PreviewArgs previewArgs = PreviewArgs();
    #endif /* PREVIEW */


    // calibration files for the left and right camera 
    // Prep Classical
    // std::string lCalibPath = std::string(SRC_ROOT_PATH).append("../calibration.xml");
    // std::string rCalibPath = std::string(SRC_ROOT_PATH).append("../calibration.xml");

    // const double baseline = 200.00;
    // cv::Ptr<cv::Feature2D> featureDetector = cv::xfeatures2d::SIFT::create();
    // cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    // ClassicalStereo classical(lCalibPath, rCalibPath, baseline, featureDetector, descriptorMatcher);

    std::cout << "created the classical stereo" << std::endl;

    //*********************************************************************************************
    // something about this section causes a segmentation fault (core dumped) error 

    // // Prep detectors
    // std::string coneRT = std::string(SRC_ROOT_PATH).append("../models/yolo4_cones_int8.rt");

    // std::cout << "prepped coneRT" << std::endl;

    // std::string keyPtsONNX = std::string(SRC_ROOT_PATH).append("../models/keypoints.onnx");

    // std::cout << "prepped keypoint onnx" << std::endl;

    // Detectors detectors;
    // detectors.initialize(coneRT, keyPtsONNX);
    
    //*********************************************************************************************

    // std::cout << "initialise the detector done" << std::endl;

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
        camera1->setup(CAMARA_NAME_L);
        camera2->setup(CAMARA_NAME_R);

        while(1) {

            camera1->startGrabbing(grabCount);
            camera2->startGrabbing(grabCount);

            int imageCount = 0;

            for (int i = 0; i < 3; i++) {
                if (camera1->isReady() & camera2->isReady()) {
                    camera1->softwareTrigger();
                    camera2->softwareTrigger();   
                }
            }

            sleep(0.5);

            //
            // if leftcamera.GetGrabResultWaitObject().Wait(0):
            //     print("Grab results wait in the left queue.")
            // if rightcamera.GetGrabResultWaitObject().Wait(0):
            //     print("Grab results wait in the right queue.")
            
            bool flag = true;

            while (imageCount < grabCount) {
                // Wait for an image and then retrieve it. A timeout of 5000 ms is used.

                // std::cout << "entered while loop" << std::endl;
                int height1;
                int width1;
                uint8_t* buffer1;

                int height2;
                int width2;
                uint8_t* buffer2;

                bool ret1 = camera1->retrieveResult(height1, width1, buffer1);
                bool ret2 = camera2->retrieveResult(height2, width2, buffer2);

                cv::Mat dst2;

                if (ret1 && ret2) {
                    cv::Mat inMat = cv::Mat(height1, width1, CV_8UC3, buffer1);
                    cv::Mat dst;
                    cv::cvtColor(inMat, dst, cv::COLOR_BGR2RGB); 
                    cv::imshow("Left camera", dst);
                    cv::waitKey(100);

                    cv::Mat inMat2 = cv::Mat(height2, width2, CV_8UC3, buffer2);
                    cv::Mat dst2;
                    cv::cvtColor(inMat2, dst2, cv::COLOR_BGR2RGB); 
                    cv::imshow("Right camera", dst2);
                    cv::waitKey(100);
                }
                imageCount++;
            }

            camera1->stopGrabbing();
            camera2->stopGrabbing();
        }
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