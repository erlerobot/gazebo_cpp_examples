#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <aruco/aruco.h>
#include <iostream>
#include <mavros/OverrideRCIn.h>
#include <math.h>
#include <mavros/State.h>

//#define FACTOR 1.2
#define FACTOR 1.2

image_transport::Subscriber sub;
ros::Subscriber mavros_state_sub;
ros::Publisher pub;
ros::Time lastTime;
std::string mode;
bool guided;
bool armed;

//Mark center
float MarkX, MarkY;
float lastMarkX, lastMarkY;

//Image center
float ImageX, ImageY;

double lastMarkerVelX, lastMarkerVelY;

double Roll, Pitch;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        double timeBetweenMarkers = (ros::Time::now() - lastTime).toSec();
        lastTime = ros::Time::now();
        
        ROS_INFO("Marker = (%f , %f) | LastMarker = (%f , %f) \n timeBetweenMarkers = %f | lastMarkerVelX = (%f , %f)\n Roll = %f | Pitch = %f\n", MarkX, MarkY, lastMarkX, lastMarkY, lastTime.toSec(), lastMarkerVelX, lastMarkerVelY, Roll, Pitch);

        aruco::MarkerDetector MDetector;
        vector<aruco::Marker> Markers;
        cv::Point2f MarkCenter;

        // Get the msg image
        cv::Mat InImage;
        InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

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

        lastMarkX = MarkX;
        lastMarkY = MarkY;

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

        if (timeBetweenMarkers < 1.0){
            lastMarkerVelX = (lastMarkX - MarkX)/timeBetweenMarkers;
            lastMarkerVelY = (lastMarkY - MarkY)/timeBetweenMarkers;
        } else{
            lastMarkerVelX = 0.0;
            lastMarkerVelY = 0.0;
        }

        
        /*if (ErX < 0){
            Roll = 1500 - log(pow(ErX,2)+1) * FACTOR;
        }else{
            Roll = 1500 + log(pow(ErX,2)+1) * FACTOR;
        }

        if (ErY < 0) {
            Pitch = 1500 - log(pow(ErY,2)+1) * FACTOR;
        }else{
            Pitch = 1500 + log(pow(ErY,2)+1) * FACTOR;
        }*/
        
        //std::cout << "Roll = " << Roll << " | Pitch = " << Pitch << "\n";

        //Roll = 1500 - ErX * FACTOR;
        //Pitch = 1500 - ErY * FACTOR;

        
        Roll = 1500 - (0.5*ErX+0.1*lastMarkerVelX);
        Pitch = 1500 - (0.5*ErY+0.1*lastMarkerVelY);  

        if (Roll > 1900)
        {
            Roll = 1900;
        } else if (Roll < 1100)
        {
            Roll = 1100;
        }

        if (Pitch > 1900)
        {
            Pitch = 1900;
        } else if (Pitch < 1100)
        {
            Pitch = 1100;
        }

        msg.channels[0] = Roll;     //Roll
        msg.channels[1] = Pitch;    //Pitch
        msg.channels[2] = 1500;     //Throttle
        msg.channels[3] = 0;        //Yaw
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

void mavrosStateCb(const mavros::StateConstPtr &msg)
{
    if(msg->mode == std::string("CMODE(0)"))
        return;
    ROS_INFO("I heard: [%s] [%d] [%d]", msg->mode.c_str(), msg->armed, msg->guided);
    mode = msg->mode;
    guided = msg->guided==128;
    armed = msg->armed==128;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    sub = it.subscribe("/erlecopter/bottom/image_raw", 1, imageCallback);
    //mavros_state_sub = nh.subscribe("/mavros/state", 1, mavrosStateCb);
    pub = nh.advertise<mavros::OverrideRCIn>("/mavros/rc/override", 10);;
    ros::spin();
}
