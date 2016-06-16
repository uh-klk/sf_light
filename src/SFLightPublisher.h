#ifndef __SFLIGHTPUBLISHER_H
#define __SFLIGHTPUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sf_light/SFLightMsg.h"

class SFLightPublisher
{
public:
    SFLightPublisher(ros::NodeHandle nh);
    ~SFLightPublisher(){}
    void init();
    void publish(sf_light::SFLightMsg msg);

protected:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    sf_light::SFLightMsg msg_;
};

#endif
