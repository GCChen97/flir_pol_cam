#include <iostream>
#include <string>
#include <array>

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "camera.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;


int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "USAGE: PCam <config path>" << std::endl;
        return 0;
    }

    PCamConfig config(argv[1]);
    
    // Spinnaker API
    SystemPtr system = System::GetInstance();
    CameraList camList = system->GetCameras();
    const unsigned int numCameras = camList.GetSize();
    cout << "Number of cameras detected: " << numCameras << endl << endl;
    // Finish if there are no cameras
    if (numCameras == 0)
    {
        // Release camera list before releasing system
        camList.Clear();
        // Release system
        system->ReleaseInstance();
        cout << "Camera not found!" << endl;
        return -1;
    }

    // Camera Instance
    PCamera imx250myr = PCamera(camList.GetByIndex(0), config);
    
    cout << endl << "*** START STREAMING ***" << endl << endl;
    ros::init(argc, argv, "PolCam");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/polcamera/raw", 1);
    
    cv::Mat image;
    ros::Rate frame_rate(config.frame_rate);
    while(nh.ok())
    {
        std_msgs::Header img_msg_header;
        img_msg_header.stamp = ros::Time::now();
        image = cv::Mat(2048, 2448, CV_8UC1, imx250myr.AcquireImage(), 2448);// GetImageData() only get pointer, not copy data
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(img_msg_header, "mono8", image).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        frame_rate.sleep();
    }
    cout << endl << "*** STOP  STREAMING ***" << endl << endl;
    imx250myr.~PCamera();

    // Spinnaker API
    // Release camera list before releasing system
    camList.Clear();
    // Release system
    system->ReleaseInstance();

    cout << endl << "-----------------------" << endl << endl;
    return 0;
    ros::shutdown();
}
