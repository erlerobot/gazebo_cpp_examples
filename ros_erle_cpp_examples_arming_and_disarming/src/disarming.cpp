#include <cstdlib>

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_disarming");
    ros::NodeHandle n;

    int rate = 10;
    ros::Rate r(rate);

    ////////////////////////////////////////////
    //////////////////DISARM////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient disarming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = false;
    if(disarming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed disarming");
    }

    while (n.ok()){
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}