#include <ros/ros.h>
#include <local_messages/Test.h>

void testCallback(const local_messages::Test::ConstPtr &msg) {
	std::cout << msg->test << std::endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_lane_detector");
	ros::NodeHandle handle;
	ros::Publisher testPublisher = handle.advertise<local_messages::Test>("test", 1000);
	ros::Rate loopRate(10);
	ros::Subscriber testSubscriber = handle.subscribe("test", 1000, testCallback);
	while (ros::ok()) {
		local_messages::Test testMessage;
		testMessage.test = 1;
		testPublisher.publish(testMessage);
		loopRate.sleep();
        ros::spinOnce();
	}
	return 0;
}
