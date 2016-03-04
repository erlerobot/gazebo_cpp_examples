#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <mavros/OverrideRCIn.h>

#define FACTOR 0.6

image_transport::Subscriber sub;
ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        aruco::MarkerDetector MDetector;
        vector<aruco::Marker> Markers;
        cv::Point2f MarkCenter;

        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        //Mark center
        float MarkX, MarkY;

        //Image center
        float ImageX, ImageY;

        // Error between Image and Mark
        float ErX = 0.0;
        float ErY = 0.0;
        
        // Get the Image center
        ImageX = InImage.cols / 2.0f;
        ImageY = InImage.rows / 2.0f;

        // Detect
        MDetector.detect(InImage,Markers);

        // Create RC msg
        mavros::OverrideRCIn msg;


        // For each marker, draw info ant its coundaries in the image
        for (unsigned int i = 0; i<Markers.size(); i++){
            Markers[i].draw(InImage,cv::Scalar(0,0,255),2);

            // Calculate the error between Image center and Mark center
            MarkCenter = Markers[i].getCenter();
            MarkX = MarkCenter.x;
            MarkY = MarkCenter.y;
            ErX = ImageX - MarkX;
            ErY = ImageY - MarkY;
        }

        msg.channels[0] = 1500 - ErX * FACTOR;
        msg.channels[1] = 1500 - ErY * FACTOR;
        msg.channels[2] = 1500;
        msg.channels[3] = 0;
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

        pub.publish(msg);

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
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
    pub = nh.advertise< mavros::OverrideRCIn >("/mavros/rc/override", 10);;
    ros::spin();
}
