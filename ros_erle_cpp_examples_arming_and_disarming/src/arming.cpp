#include <cstdlib>

#include <ros/ros.h>
#include <mavros/CommandBool.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mavros_arming");
    ros::NodeHandle n;

    int rate = 10;
    ros::Rate r(rate);

    ////////////////////////////////////////////
    ///////////////////ARM//////////////////////
    ////////////////////////////////////////////
    ros::ServiceClient arming_cl = n.serviceClient<mavros::CommandBool>("/mavros/cmd/arming");
    mavros::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_ERROR("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming");
    }

    while (n.ok()){
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}