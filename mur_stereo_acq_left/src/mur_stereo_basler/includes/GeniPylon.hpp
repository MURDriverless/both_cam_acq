/*
related to BaslarDriver. It seems that Andrew took the stuff from BaslarDriver and
made a class from it. 

*/

#pragma once
#include "GeniWrap.hpp"

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/ParameterIncludes.h>
#include <pylon/EnumParameterT.h>
#include <GenApi/GenApi.h>

static const int defaultExposureTime = 5000;

class PylonCam: public IGeniCam {
    private:
        Pylon::CBaslerUniversalInstantCamera camera;
        Pylon::CGrabResultPtr ptrGrabResult;

    public:
        PylonCam() {

        }
        
        void initializeLibrary() {
            Pylon::PylonInitialize();
        }

        void finalizeLibrary() {
            Pylon::PylonTerminate();
        }

        void setup(const std::string cameraName, int lightSetting, int exposureTimeSetting, int frameRateSetting) {
            Pylon::CTlFactory& TlFactory = Pylon::CTlFactory::GetInstance();
            Pylon::CDeviceInfo di;
            di.SetFriendlyName(cameraName.c_str());
            camera.Attach(TlFactory.CreateDevice(di));

            // Print the model name of the camera.
            std::cout << "Using device " << camera.GetDeviceInfo().GetModelName() << std::endl;

            // The parameter MaxNumBuffer can be used to control the count of buffers
            // allocated for grabbing. The default value of this parameter is 10.
            camera.MaxNumBuffer = 1;

            camera.Open();

            GenApi::INodeMap& nodemap = camera.GetNodeMap();

            // captures only the bottom half of the images 
            // camera.OffsetY.SetValue(600);
            camera.Width = 1920; 
            camera.Height = 1200; 

            // setting the pixelformat removed the 3x3 grid problem when recording images and it allows us to record in colored! 
            Pylon::CEnumParameter(nodemap, "PixelFormat").SetValue("RGB8");
            
            Pylon::CBooleanParameter(nodemap, "AcquisitionFrameRateEnable").SetValue(true);

            if (frameRateSetting == 1) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(160.0);
            }
            else if (frameRateSetting == 2) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(5.0);
            }
            else if (frameRateSetting == 3) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(10.0);
            }
            else if (frameRateSetting == 4) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(15.0);
            }
            else if (frameRateSetting == 5) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(20.0);
            }
            else if (frameRateSetting == 6) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(25.0);
            }
            else if (frameRateSetting == 7) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(30.0);
            }
            else if (frameRateSetting == 8) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(35.0);
            }
            else if (frameRateSetting == 9) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(40.0);
            }
            else if (frameRateSetting == 10) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(45.0);
            }
            else if (frameRateSetting == 11) {
                Pylon::CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(50.0);
            }

            if (lightSetting == 1) {
                Pylon:: CEnumParameter(nodemap, "BslLightSourcePreset").SetValue("Off");
                std::cout << "light preset: off" << std::endl;
            }
            else if (lightSetting == 2) {
                Pylon:: CEnumParameter(nodemap, "BslLightSourcePreset").SetValue("Daylight5000K");
                std::cout << "daylight 5000K" << std::endl;
            }
            else if (lightSetting == 3) {
                Pylon:: CEnumParameter(nodemap, "BslLightSourcePreset").SetValue("Daylight6500K");
                // camera.LightSourcePreset.SetValue(Basler_UniversalCameraParams::LightSourcePresetEnums::LightSourcePreset_Daylight6500K);
                std::cout << "daylight 6500K" << std::endl;
            }
  
            if (exposureTimeSetting == 1) {
                // Set the exposure time to 3500 microseconds
                Pylon:: CEnumParameter(nodemap, "ExposureMode").SetValue("Timed");
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(defaultExposureTime);
                // Pylon:: CEnumParameter(nodemap, "ExposureAuto").SetValue("Continuous");
            } 
            else if (exposureTimeSetting == 2) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(4000);
            }
            else if (exposureTimeSetting == 3) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(3000);
            }
            else if (exposureTimeSetting == 4) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(2000);
            }
            else if (exposureTimeSetting == 5) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(1000);
            }
            else if (exposureTimeSetting == 6) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(500);
            }
            else if (exposureTimeSetting == 7) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(250);
            }
            else if (exposureTimeSetting == 8) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(125);
            }
            else if (exposureTimeSetting == 9) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(100);
            }
            else if (exposureTimeSetting == 10) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(75);
            }
            else if (exposureTimeSetting == 11) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(50);
            }
            else if (exposureTimeSetting == 12) {
                Pylon:: CFloatParameter(nodemap, "ExposureTime").SetValue(25);
            }
            

            // Set the measuring location to core board
            // can't measure any other points on the camera. 
            Pylon::CEnumParameter(nodemap, "DeviceTemperatureSelector").SetValue("Coreboard");

            // // Select the Frame Start trigger
            // Pylon::CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");
            // // Enable triggered image acquisition for the Frame Start trigger
            // Pylon::CEnumParameter(nodemap, "TriggerMode").SetValue("On");
            // // Set the trigger source for the Frame Start trigger to Software
            // Pylon::CEnumParameter(nodemap, "TriggerSource").SetValue("Software");

         

            /** 
            // software triggering code 
            // Select the Frame Start trigger
            // camera.TriggerSelector.SetValue(Basler_UniversalCameraParams::TriggerSelector_FrameStart);
            // // Enable triggered image acquisition for the Frame Start trigger
            // camera.TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_On);
            // // Set the trigger source for the Frame Start trigger to Software
            // camera.TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_Software);

            INodeMap& nodemap = camera.GetNodeMap();
            // Select the Frame Start trigger
            Pylon::CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");
            // Enable triggered image acquisition for the Frame Start trigger
            Pylon::CEnumParameter(nodemap, "TriggerMode").SetValue("On");
            // Set the trigger source for the Frame Start trigger to Software
            Pylon::CEnumParameter(nodemap, "TriggerSource").SetValue("Software");

            **/
     
            // Enable Balance White Auto by setting the operating mode to Continuous
            // Pylon::CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue("Continuous");
            camera.Close();
        }

        void startGrabbing(uint32_t numImages = UINT32_MAX) {
            // Start the grabbing of c_countOfImagesToGrab images.
            // The camera device is parameterized with a default configuration which
            // sets up free-running continuous acquisition.
            camera.StartGrabbing(numImages, Pylon::GrabStrategy_LatestImageOnly);
        }

        void stopGrabbing() {
            camera.StopGrabbing();
        }

        bool isGrabbing() {
            return camera.IsGrabbing();
        }

        bool isReady() {
            // Select and enable the Frame Start trigger
            camera.TriggerSelector.SetValue(Basler_UniversalCameraParams::TriggerSelector_FrameStart);
            camera.TriggerMode.SetValue(Basler_UniversalCameraParams::TriggerMode_On);
            camera.TriggerSource.SetValue(Basler_UniversalCameraParams::TriggerSource_Software);

            return camera.WaitForFrameTriggerReady(5000, Pylon::TimeoutHandling_ThrowException);
        }

        double getFrameRate() {
            return camera.ResultingFrameRate.GetValue();
        }

        double getExposureTime() {
            return camera.ExposureTime.GetValue();  
        }

        void softwareTrigger() {
            camera.ExecuteSoftwareTrigger();
        }

        double getTemperatureReading() {
            return camera.DeviceTemperature.GetValue();
        }

        bool retrieveResult(int &height, int &width, uint8_t* &buffer) {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            height = ptrGrabResult->GetHeight();
            width  = ptrGrabResult->GetWidth();
            buffer = static_cast<uint8_t *>(ptrGrabResult->GetBuffer());

            return ptrGrabResult->GrabSucceeded();
        }

        void clearResult() {
            return;
        }
};