#ifndef __SFLIGHTSUBSCRIBER_H
#define __SFLIGHTSUBSCRIBER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "sf_light/SFLightMsg.h" //msg
#include "ArduinoSerialCom.h"


typedef union {
    float f;
    unsigned char b[4];
} float2bytes;

class SFLightSubscriber
{
    
public: 
  SFLightSubscriber(char *portname, int baudrate, ros::NodeHandle nh) : Arduino_(portname, baudrate), node_(nh){}
  ~SFLightSubscriber() {}
  
  void init();
  
  void receiveCB(const sf_light::SFLightMsg::ConstPtr& msg);
  
  void setArduinoPort(char* portname) { 
  	serialPort_ = portname;
  	Arduino_.setPort(serialPort_);
  }
	
  void setArduinoBaud(int baudrate){
  	baudrate_ = baudrate;
  	Arduino_.setBaud(baudrate_);
  }

protected:
	ros::NodeHandle node_;
  ros::Subscriber subscriber_;
	sf_light::SFLightMsg msg_;
	char *serialPort_;
  int baudrate_;
	ArduinoSerialCommunication Arduino_;
	
};


#endif
