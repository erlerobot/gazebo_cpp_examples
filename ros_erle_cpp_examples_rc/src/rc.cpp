#include <cstdlib>
#include <stdlib.h>     /* atoi */

#include <ros/ros.h>
#include <mavros_msgs/OverrideRCIn.h>

int main(int argc, char **argv)
{
    if(argc<5){
        printf("Usage: rosrun ros_erle_cpp_examples_rc rc pitch roll throttle yaw\n");
        return -1;
    }

    ros::init(argc, argv, "mavros_rc_override");
    ros::NodeHandle n;

    int pitch = atoi(argv[1]);
    int roll = atoi(argv[1]);
    int throttle = atoi(argv[1]);
    int yaw = atoi(argv[1]);

    int rate = 100;
    ros::Rate r(rate);

    ros::Publisher rc_override_pub = n.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
    
    mavros_msgs::OverrideRCIn msg_override;

    while (n.ok()){
   
        msg_override.channels[0] = roll;
        msg_override.channels[1] = pitch;
        msg_override.channels[2] = throttle;
        msg_override.channels[3] = yaw;
        msg_override.channels[4] = 1100;
        msg_override.channels[5] = 1100;
        msg_override.channels[6] = 1100;
        msg_override.channels[7] = 1100;

        rc_override_pub.publish(msg_override);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
