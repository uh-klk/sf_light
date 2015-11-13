
#include "SFLightSubscriber.h"

void SFLightSubscriber::init()
{
    /**
    * The subscribe() call is how you tell ROS that you want to receive messages
    * on a given topic.  This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing.  Messages are passed to a callback function, here
    * called receiveCallback.  subscribe() returns a Subscriber object that you
    * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
    * object go out of scope, this callback will automatically be unsubscribed from
    * this topic.
    *
    * The second parameter to the subscribe() function is the size of the message
    * queue.  If messages are arriving faster than they are being processed, this
    * is the number of messages that will be buffered up before beginning to throw
    * away the oldest ones.
    */
   
    subscriber_ = node_.subscribe("SF_Light", 10, &SFLightSubscriber::receiveCB, this);
}

void SFLightSubscriber::receiveCB(const sf_light::SFLightMsg::ConstPtr& msg)
{
	static unsigned char data[13] = {0};
	static float2bytes convertFloat2Bytes;

	ROS_INFO("I heard mode: [%d]", msg->mode);
  ROS_INFO("I heard frequency: [%f]", msg->frequency);
  ROS_INFO("I heard priority: [%d]", msg->priority);
  ROS_INFO("I heard color: [%d, %d, %d]", (int) msg->color.r, (int) msg->color.g, (int) msg->color.b);
  ROS_INFO("I heard on pct: [%f]", msg->on_pct);

  /*data*/
  data[0] = (unsigned char) msg->mode;

  //convert frequency from float to bytes
  convertFloat2Bytes.f = msg->frequency;
  for (int i=0; i<4; i++)
  	data[1+i] = convertFloat2Bytes.b[i];
    
  data[5] = (unsigned char) msg->color.r;
  data[6] = (unsigned char) msg->color.g;
  data[7] = (unsigned char) msg->color.b;
  //data[8] = (unsigned char) msg->color.a;

	//convert on_pct from float to bytes
	convertFloat2Bytes.f = msg->on_pct;
	for (int i=0; i<4; i++)
    data[8+i] = convertFloat2Bytes.b[i];

	/*
  data[9] = (unsigned char)((hold_time >> 24) & 0xFF);
  data[10] = (unsigned char)((hold_time >> 16) & 0xFF);
  data[11] = (unsigned char)((hold_time >> 8) & 0xFF);
  data[12] = (unsigned char)((hold_time) & 0xFF);
  */
  
	printf("\n ++++++++++++++++++++++++++++++++++++++++++");	
	printf("\nPayload is: "); 
 
  for (int i =0; i < sizeof(data); i++)
  	printf("%d, ", data[i]);
 
  printf("\n ++++++++++++++++++++++++++++++++++++++++++");	
  
  if (Arduino_.openPort())
	{	
		printf("\n Port open successfully");
		//usleep(1000*1000);
		printf("\nSending data");
		Arduino_.sendData(data, sizeof(data));
		Arduino_.closePort();
		//ros::shutdown();  //sending just 1 package for debuging with Arduino		
	}
	else printf("\nFailed to open port"); 

}


