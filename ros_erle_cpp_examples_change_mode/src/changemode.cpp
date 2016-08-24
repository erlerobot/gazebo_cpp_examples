#include <cstdlib>

#include <ros/ros.h>
#include <mavros/SetMode.h>

int main(int argc, char **argv)
{
    if(argc<2){
        printf("Usage: rosrun ros_erle_cpp_examples_change_mode changemode <mode>\n");
        return -1;
    }

    ros::init(argc, argv, "mavros_change_mode");
    ros::NodeHandle n;

    int rate = 10;
    ros::Rate r(rate);

    ros::ServiceClient cl = n.serviceClient<mavros::SetMode>("/mavros/set_mode");
    mavros::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = argv[1];
    if(cl.call(srv_setMode)){
        ROS_ERROR("setmode send ok %d value:", srv_setMode.response.success);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    while (n.ok()){
      ros::spinOnce();
      r.sleep();
    }

    return 0;
}