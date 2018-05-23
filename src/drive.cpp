#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

ros::Publisher drive_pub;
ros::Publisher dig_pub;
ros::Publisher halt_pub_front_left;
ros::Publisher halt_pub_back_left;
ros::Publisher halt_pub_back_right;

bool digging_state = false;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    bool new_digging_state = joy_msg->buttons[4];
    if (!digging_state && new_digging_state) {
        std::cout << "...and now we're digging!" << std::endl;
    } else if (digging_state && !new_digging_state) {
        std::cout << "...and now we're driving, you absolute scrub." << std::endl;
    }
    digging_state = new_digging_state;
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
    if (digging_state) {
        geometry_msgs::Twist dig_vel, halt_vel;
        dig_vel.linear.y = cmd_vel->linear.y;
        dig_pub.publish(dig_vel);
        halt_pub_front_left.publish(halt_vel);
        halt_pub_back_left.publish(halt_vel);
        halt_pub_back_right.publish(halt_vel);
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
