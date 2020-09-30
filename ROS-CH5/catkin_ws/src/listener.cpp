#include "ros/ros.h"
#include "std_msgs/String.h"

//Subscribing and registering a callback function for the topic,
//chatterCallback. The callback is defined at the beginning of
//the code. Whenever a message comes to the "chatter" topic, this
//callback is executed. Inside the callback, the data in the 
//message is printed.

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	ros::spin();
	return 0;
}
