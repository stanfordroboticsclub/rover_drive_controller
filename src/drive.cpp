#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

ros::Publisher drive_pub;
bool digging_state = false;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    digging_state = joy_msg->buttons[4];
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    if (digging_state) {
        // pass
    } else {
        drive_pub.publish(*cmd_vel);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_controller");
    ros::NodeHandle n;

    ros::Subscriber joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 1000, joyCallback);
    ros::Subscriber twist_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1000, twistCallback);
    drive_pub = n.advertise<geometry_msgs::Twist>("/rover_drive_controller/cmd_vel", 1000);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
