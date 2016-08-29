#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include <time.h>

double x, y, z;
double qx, qy, qz, qw;

double x_amcl, y_amcl, z_amcl;
double qx_amcl, qy_amcl, qz_amcl, qw_amcl;

ros::Publisher odom_pub;
ros::Publisher odom_amcl_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry msg_to_send;
    //printf("pose2D: %.2f %.2f %.2f Q: %.2f %.2f %.2f %.2f\n", x, y, z, qx, qy, qz, qw);
    msg_to_send.pose.pose.position.x = msg->pose.pose.position.x;
    msg_to_send.pose.pose.position.y = msg->pose.pose.position.y;
    msg_to_send.pose.pose.position.z = msg->pose.pose.position.z;

    msg_to_send.child_frame_id = "base_footprint";
    msg_to_send.header.frame_id = "odom";
    msg_to_send.header.stamp = ros::Time::now();

    msg_to_send.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    msg_to_send.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    msg_to_send.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    msg_to_send.pose.pose.orientation.w = msg->pose.pose.orientation.w;

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;

    odom_pub.publish(msg_to_send);

    x_amcl = msg->pose.pose.position.x;
    y_amcl = msg->pose.pose.position.y;
    z_amcl = msg->pose.pose.position.z;

    qx_amcl = msg->pose.pose.orientation.x;
    qy_amcl = msg->pose.pose.orientation.y;
    qz_amcl = msg->pose.pose.orientation.z;
    qw_amcl = msg->pose.pose.orientation.w;
    odom_amcl_pub.publish(msg_to_send);

}

void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    nav_msgs::Odometry msg_to_send;
    msg_to_send.pose.pose.position.x = msg->pose.pose.position.x;
    msg_to_send.pose.pose.position.y = msg->pose.pose.position.y;
    msg_to_send.pose.pose.position.z = msg->pose.pose.position.z;
    
    msg_to_send.child_frame_id = "base_footprint";
    msg_to_send.header.frame_id = "odom";
    msg_to_send.header.stamp = ros::Time::now();

    msg_to_send.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    msg_to_send.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    msg_to_send.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    msg_to_send.pose.pose.orientation.w = msg->pose.pose.orientation.w;

//    odom_amcl_pub.publish(msg_to_send);
    
    printf("%.2f %.2f %.2f Q: %.2f %.2f %.2f %.2f\n",
    		 x_amcl,
    		 y_amcl,
    		 z_amcl,
    		 qx_amcl,
    		 qy_amcl,
    		 qz_amcl,
    		 qw_amcl);
    x_amcl = msg->pose.pose.position.x;
    y_amcl = msg->pose.pose.position.y;
    z_amcl = msg->pose.pose.position.z;

    qx_amcl = msg->pose.pose.orientation.x;
    qy_amcl = msg->pose.pose.orientation.y;
    qz_amcl = msg->pose.pose.orientation.z;
    qw_amcl = msg->pose.pose.orientation.w;
    
}

int main(int argc, char* argv[])
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    qx =0.0;
    qy = 0.0;
    qz = 0.0;
    qw = 1.0;

    x_amcl = 0.0;
    y_amcl = 0.0;
    z_amcl = 0.0;
    qx_amcl =0.0;
    qy_amcl = 0.0;
    qz_amcl = 0.0;
    qw_amcl = 1.0;

    ros::init(argc, argv, "ros_erle_rover_tf");
    ros::NodeHandle n;

    int32_t publish_rate_ = 50;
    tf::TransformBroadcaster tf_br_;
    tf::StampedTransform tf_world_to_map_;
    tf::StampedTransform tf_map_to_odom_;
    tf::StampedTransform tf_map_to_odomacml_;
    tf::StampedTransform tf_odom_to_base_;
    tf::StampedTransform tf_odomamcl_to_baseamcl_;
    tf::StampedTransform tf_base_to_baselink_;
    tf::StampedTransform tf_base_to_chassis_;
    tf::StampedTransform tf_chassis_to_sonar_;
    tf::StampedTransform tf_sonar_to_laser_;

    // set up parent and child frames
    tf_world_to_map_.frame_id_ = std::string("world");
    tf_world_to_map_.child_frame_id_ = std::string("map");

    // set up parent and child frames
    tf_map_to_odom_.frame_id_ = std::string("map");
    tf_map_to_odom_.child_frame_id_ = std::string("odom");

    tf_map_to_odomacml_.frame_id_ = std::string("map");
    tf_map_to_odomacml_.child_frame_id_ = std::string("odom_amcl");

    // set up parent and child frames
    tf_odom_to_base_.frame_id_ = std::string("odom");
    tf_odom_to_base_.child_frame_id_ = std::string("base_footprint");

    tf_odomamcl_to_baseamcl_.frame_id_ = std::string("odom_amcl");
    tf_odomamcl_to_baseamcl_.child_frame_id_ = std::string("base_footprint_amcl");

    tf_base_to_baselink_.frame_id_ = std::string("base_footprint");
    tf_base_to_baselink_.child_frame_id_ = std::string("base_link");

    // set up parent and child frames
    tf_base_to_chassis_.frame_id_ = std::string("base_link");
    tf_base_to_chassis_.child_frame_id_ = std::string("chassis");

    tf_chassis_to_sonar_.frame_id_ = std::string("chassis");
    tf_chassis_to_sonar_.child_frame_id_ = std::string("sonar2_link");

    tf_sonar_to_laser_.frame_id_ = std::string("sonar2_link");
    tf_sonar_to_laser_.child_frame_id_ = std::string("laser");

    //ros::Subscriber sub = n.subscribe("odom", 10, odomCallback);
    ros::Subscriber sub2 = n.subscribe("amcl_pose", 10, amclCallback);
    ros::Subscriber sub3 = n.subscribe("/rover/ground_truth/odometry", 10, odomCallback);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
    odom_amcl_pub = n.advertise<nav_msgs::Odometry>("odom_amcl", 10);

    // set up publish rate
    ros::Rate loop_rate(publish_rate_);

    // main loop
    while (ros::ok())
    {
        struct timespec tim, tim2;
        tim.tv_sec = 0;
        tim.tv_nsec = 50;

        // time stamp
        tf_world_to_map_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_map_to_odom_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_map_to_odomacml_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_odom_to_base_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_odomamcl_to_baseamcl_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_base_to_chassis_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_chassis_to_sonar_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_sonar_to_laser_.stamp_ = ros::Time::now(); //nanosleep(&tim , &tim2);
        tf_base_to_baselink_.stamp_ = ros::Time::now();

        // specify actual transformation vectors from odometry
        // NOTE: zeros have to be substituted with actual variable data
        tf_map_to_odom_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_map_to_odom_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));

        tf_world_to_map_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_world_to_map_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));

        tf_map_to_odomacml_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_map_to_odomacml_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));

        // broadcast transform
        tf_br_.sendTransform(tf_map_to_odom_);
        tf_br_.sendTransform(tf_map_to_odomacml_);
        tf_br_.sendTransform(tf_world_to_map_);

        // time stamp
        // specify actual transformation vectors from odometry
        // NOTE: zeros have to be substituted with actual variable data
        tf_odom_to_base_.setOrigin(tf::Vector3(x, y, z));
        tf_odom_to_base_.setRotation(tf::Quaternion(qx, qy, qz, qw));

        tf_odomamcl_to_baseamcl_.setOrigin(tf::Vector3(x_amcl, y_amcl, z_amcl));
        tf_odomamcl_to_baseamcl_.setRotation(tf::Quaternion(qx_amcl, qy_amcl, qz_amcl, qw_amcl));

        // broadcast transform
        tf_br_.sendTransform(tf_odom_to_base_);
        tf_br_.sendTransform(tf_odomamcl_to_baseamcl_);

        tf_base_to_chassis_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_base_to_chassis_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
        tf_br_.sendTransform(tf_base_to_chassis_);

        tf_sonar_to_laser_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_sonar_to_laser_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
        tf_br_.sendTransform(tf_sonar_to_laser_);

        tf_chassis_to_sonar_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_chassis_to_sonar_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
        tf_br_.sendTransform(tf_chassis_to_sonar_);

        tf_base_to_baselink_.setOrigin(tf::Vector3(0.0f, 0.0f, 0.0f));
        tf_base_to_baselink_.setRotation(tf::Quaternion(0.0f, 0.0f, 0.0f, 1.0f));
        tf_br_.sendTransform(tf_base_to_baselink_);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}
