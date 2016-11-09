#include <ros/ros.h>
#include "SFLightPublisher.h"
#include "sf_light/SFLightMsg.h"


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "publisher");

    ros::NodeHandle nodePub;


    SFLightPublisher sf_lightPub(nodePub);
  
    sf_lightPub.init();

    unsigned char data[13];
    long hold_time = 100;
    long cross_time = 50;
 
    sf_light::SFLightMsg msg; //the message type
 	
    msg.mode = 3;
    msg.frequency = 0.5; //how many sine wave cycle (i.e.on-off) per sec.
    msg.priority = 10;
    msg.color.r = 255;
    msg.color.g = 255;
    msg.color.b = 255;
    msg.on_pct = 1; //how long the on should be in % per cycle
 
    ros::Rate loop_rate(0.2);
 	
    while (ros::ok())
    {
        sf_lightPub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
