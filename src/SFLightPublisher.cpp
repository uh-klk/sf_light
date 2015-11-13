#include "SFLightPublisher.h"


SFLightPublisher::SFLightPublisher(ros::NodeHandle nh)
{
    node_ = nh;
}


void SFLightPublisher::init()
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
    
    publisher_ = node_.advertise<sf_light::SFLightMsg>("SF_Light", 1000);
}

void SFLightPublisher::publish(sf_light::SFLightMsg msg)
{
	/*
	ROS_INFO("I heard mode: [%d]", msg->mode);
  ROS_INFO("I heard frequency: [%f]", msg->frequency);
  ROS_INFO("I heard priority: [%d]", msg->priority);
  ROS_INFO("I heard color: [%d, %d, %d, %d]", (int) msg->color.r, (int) msg->color.g, (int) msg->color.b, (int) msg->color.a );
  ROS_INFO("I heard hold_time: [%f]", msg->hold_time);
  ROS_INFO("I heard cross_time: [%f]", msg->cross_time);
*/
	msg_.mode = msg.mode;
	msg_.frequency = msg.frequency;
	msg_.priority = msg.priority;
	msg_.color = msg.color;
	msg_.on_pct = msg.on_pct;
	ROS_INFO("Publishing");
	publisher_.publish(msg_);
}


