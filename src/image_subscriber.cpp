#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        aruco::MarkerDetector MDetector;
        vector<aruco::Marker> Markers;

        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Detect
        MDetector.detect(InImage,Markers);
        // For each marker, draw info ant its coundaries in the image
        for (unsigned int i = 0; i<Markers.size(); i++){
            cout<<Markers[i]<<endl;
            Markers[i].draw(InImage,cv::Scalar(0,0,255),2);
        }

        cv::imshow("view", InImage);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    // cv::namedWindow("view");
    // cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
    ros::spin();
    //cv::destroyWindow("view");
}
