#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <mavros/OverrideRCIn.h>

/*
    TO DO : Choose a FACTOR of your choice. Note that:
            * Increasing this value will make the drone move faster, but imprecisely.
            * Decreasing the value will make the drone move more precisely, but slower.
*/
#define FACTOR // TO DO


/*
    TO DO : Choose the correct base values for roll, pitch and yaw
*/
#define BASE_ROLL_VALUE // TO DO
#define BASE_PITCH_VALUE //TO DO
#define BASE_YAW_VALUE // TO DO

image_transport::Subscriber sub;
ros::Publisher pub;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    aruco::MarkerDetector MDetector;
    vector<aruco::Marker> Markers;
    cv::Point2f MarkCenter;

    /*
        TO DO : Extract the image from the message (msg) and store it in InImage
    */
    cv::Mat InImage /* = image_from_the_msg */;

    // Detect the markers in InImage and store them in Markers
    MDetector.detect(InImage,Markers);

    // Create RC msg
    mavros::OverrideRCIn msg;

    // For each marker, draw info ant its coundaries in the image
    for (unsigned int i = 0; i<Markers.size(); i++){
        Markers[i].draw(InImage,cv::Scalar(0,0,255),2);

        /*
            TO DO : Calculate the error between the center of the image and the center of the mark

            ERROR_X_AXIS = IMAGE_CENTER_X_AXIS - MARK_CENTER_X_AXIS
            ERROR_Y_AXIS = IMAGE_CENTER_Y_AXIS - MARK_CENTER_Y_AXIS

        */
    }

    /*  
        TO DO : For each RC channel aply the desired values so that the center of the camera matches with the center of the mark.

        msg.channels[0] = BASE_ROLL_VALUE - ERROR_X_AXIS * FACTOR;  // Override roll value
        msg.channels[1] = BASE_PITCH_VALUE - ERROR_Y_AXIS * FACTOR; // Override pitch value
        msg.channels[2] = BASE_YAW_VALUE;                           // Override yaw value
        msg.channels[3] = 0;                                        // Override throttle value
        msg.channels[4] = 0;
        msg.channels[5] = 0;
        msg.channels[6] = 0;
        msg.channels[7] = 0;

    */

    // Publish the msg
    pub.publish(msg);

    // Shows the image on screen
    cv::imshow("view", InImage);
    cv::waitKey(30);

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
