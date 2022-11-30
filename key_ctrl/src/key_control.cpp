#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

ros::Publisher speed_cmd_publisher;
static float prev_linear_spd = 0;
static float prev_angular_spd = 0;
static float this_linear_spd = 0;
static float this_angular_spd = 0;

static const float FORWARD_SPD = 0.5;
static const float ANGULAR_TURN = 0.5;

void keyboard_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    geometry_msgs::Twist speed_cmd;
    this_linear_spd = (msg.get()->linear).x;
    this_angular_spd = (msg.get()->angular).z;

    // if (this_linear_spd > prev_linear_spd)
    //     speed_cmd.linear.x = FORWARD_SPD;
    // else if (this_linear_spd < prev_linear_spd)
    //     speed_cmd.linear.x = -FORWARD_SPD;
    // if (this_angular_spd > prev_angular_spd)
    //     speed_cmd.angular.z = ANGULAR_TURN;
    // else if (this_angular_spd < prev_angular_spd)
    //     speed_cmd.angular.z = -ANGULAR_TURN;

    speed_cmd.linear.x = this_linear_spd;
    speed_cmd.angular.z = this_angular_spd;

    // std::cout << "Linear: " << speed_cmd.linear << std::endl;
    // std::cout << "Angular: " << speed_cmd.angular << std::endl;
    speed_cmd_publisher.publish(speed_cmd);

    prev_linear_spd = this_linear_spd;
    prev_angular_spd = this_angular_spd;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_control");
    ros::NodeHandle key_control_node_handle;

    speed_cmd_publisher = key_control_node_handle.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);

    ros::Subscriber subscribe_keyboard = key_control_node_handle.subscribe("/cmd_vel", 1, keyboard_callback);
    ros::spin();

    return 0;
}