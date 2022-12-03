#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"

ros::Publisher speed_cmd_publisher;
ros::Publisher camera_enable_publisher;
ros::Publisher laser_enable_publisher;

static float this_linear_spd = 0;
static float this_angular_spd = 0;
static bool enable_laser = true;
static bool enable_camera = true;
static bool manual_control = true;

void keyboard_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    geometry_msgs::Twist speed_cmd;
    std_msgs::Bool laser_enable, camera_enable;
    this_linear_spd = (msg.get()->linear).x;
    this_angular_spd = (msg.get()->angular).z;

    enable_camera = (msg.get()->angular).x == 1 ? true : false;
    enable_laser = (msg.get()->angular).y == 1 ? true : false;
    manual_control = (msg.get()->linear).y == 1 ? true : false;

    speed_cmd.linear.x = this_linear_spd;
    speed_cmd.angular.z = this_angular_spd;

    laser_enable.data = enable_laser;
    camera_enable.data = enable_camera;

    // std::cout << "Linear: " << speed_cmd.linear << std::endl;
    // std::cout << "Angular: " << speed_cmd.angular << std::endl;
    if (manual_control)
    {   
        ROS_INFO_STREAM("In manual control mode");
        speed_cmd_publisher.publish(speed_cmd);
    }
    else
    {
        ROS_INFO_STREAM("In auto tracking mode. Turnning of the laser is suggested!");
    }
    camera_enable_publisher.publish(camera_enable);
    laser_enable_publisher.publish(laser_enable);

    if (enable_laser)
        ROS_INFO_STREAM("Laser is Enabled");
    else
        ROS_INFO_STREAM("Laser is Disabled");
    if (enable_camera)
        ROS_INFO_STREAM("Camera is Enabled");
    else
        ROS_INFO_STREAM("Camera is Disabled");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "key_control");
    ros::NodeHandle key_control_node_handle;

    speed_cmd_publisher = key_control_node_handle.advertise<geometry_msgs::Twist>("/vrep/cmd_vel", 1);
    camera_enable_publisher = key_control_node_handle.advertise<std_msgs::Bool>("/vrep/camera_switch", 1);
    laser_enable_publisher = key_control_node_handle.advertise<std_msgs::Bool>("/vrep/laser_switch", 1);

    ros::Subscriber subscribe_keyboard = key_control_node_handle.subscribe("/cmd_vel", 1, keyboard_callback);
    ros::spin();

    return 0;
}