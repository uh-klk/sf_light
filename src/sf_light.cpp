#include <ros/ros.h>
#include <std_msgs/String.h>
#include "SFLightSubscriber.h"
#include "sf_light/SFLightMsg.h" //msg
#include "ArduinoSerialCom.h"
#include <unistd.h>


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "sf_light");
    ros::NodeHandle nodeSub;

    //Serial Comm
    char * serialPort = "/dev/ttyACM0";
    int baud = B115200;
    SFLightSubscriber sf_lightSub(serialPort, baud, nodeSub);	//setup serial connection to Arduino and send in ros's nodeHandle

    sf_lightSub.init(); //subscribe to topic and setup the call back function.

    ros::Rate loop_rate(10);
 	
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
	
    return 0;
}
