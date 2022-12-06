#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <cmath>

#define PI 3.14159265358

using namespace std;
using namespace cv;
using namespace sensor_msgs;

const int Resolution = 512; // for camera
const float Integral_Limit = 50; 

static ros::Publisher tracking_pub;
static ros::Subscriber tracking_sub;
static ros::Subscriber msg_sub;
// this publisher is for rqt plot to tune the pid parameters
static ros::Publisher error_pub;
static bool manual_control = 1;
static bool enable_laser;
static double last_tracking_time = -1; // help to determine whether the robot is keep tracking or start tracking


struct PID{
    float kp;
    float ki;
    float kd;
    float integral_part;
    float last_error;
};


static PID linear_pid = {0};
static PID angular_pid = {0};

// parameters
const float L_P = 0.016;
const float L_D = 0.02;
const float L_I = 0.0;
const float A_P = 0.025;
const float A_D = 0.020;
const float A_I = 0.0;
const float COMPENSATION_FACTOR = 0.2;


float pid_cal(PID& pid, float error, double duration)
{
    float output = 0;
    // if (pid.last_error != 0)
    // {
    //     if (abs((error - pid.last_error) / duration) > tan(70 * PI / 180.0f))
    //         error = pid.last_error;
    // }
    output += pid.kp * error;
    output += pid.kd * (error - pid.last_error) / duration;
    pid.integral_part += pid.ki * duration * error;
    if (pid.integral_part > Integral_Limit)
        pid.integral_part = Integral_Limit;
    output += pid.integral_part;
    pid.last_error = error;
    // cout<<"pid out: "<<output<<endl;
    return output;
}


void message_callback(const geometry_msgs::Twist::ConstPtr& msg)
{   
    //wy->laser; vy->control mode
    enable_laser = (msg.get()->angular).y == 1 ? true : false;
    manual_control = (msg.get()->linear).y == 1 ? true : false;
}


void image_callback(const ImageConstPtr& frame)
{
    if (manual_control)
    {
        last_tracking_time = -1;
        return ;
    }

    cv_bridge::CvImageConstPtr cv_image;
    // convert the ros frame to cv image with new copy
    try
    {
        cv_image = cv_bridge::toCvCopy(frame, image_encodings::BGR8);
    }
    catch(cv_bridge::Exception e)
    {
        ROS_ERROR("Cannot convert the ros image to the cv image due to %s", e.what());
        linear_pid.integral_part = 0;
        angular_pid.integral_part = 0;
        last_tracking_time = -1;
        return ;
    }
    // masked image to show the yellow ball
    Mat original_image = cv_image->image;
    flip(original_image, original_image, 1);
    Mat hsv;
    cvtColor(original_image, hsv, COLOR_BGR2HSV);
    Scalar yellow_lower_bound = Scalar(26, 43, 46);
    Scalar yellow_upper_bound = Scalar(34, 255, 255);
    Mat masked_image;
    inRange(hsv, yellow_lower_bound, yellow_upper_bound, masked_image);

    vector<vector<Point>> contours;
    // Extract contours with cascade level and simple chain approximation
    findContours(masked_image, contours, 3, 2);
    Mat contours_img;
    int num_contours = contours.size();
    if (num_contours < 1) 
    {
        // ROS_INFO_STREAM("no contours found");
        if (!manual_control) {
            linear_pid.integral_part = 0;
            angular_pid.integral_part = 0;
            last_tracking_time = -1;
            return ;
        }
    }else
    {
        original_image.copyTo(contours_img);
        drawContours(contours_img, contours, -1, Scalar(255, 255, 255));
        
    }
    int width = masked_image.size().width;
    int height = masked_image.size().height;
    Mat pointsf;
    RotatedRect box;
    int num_circle = 0;
    int circle_edges = 0;
    // find most like circle
    for (int i = 0; i < num_contours; i++) 
    {
        // check whether the contour is circle
        float perimere = arcLength(contours[i], true);
        vector<Point> approximation; 
        approxPolyDP(Mat(contours[i]), approximation, perimere * 0.02, true);
        // assume the contour has more than 12 edges is a circle
        if (approximation.size() > 7 && approximation.size() < 9)
        {    
            // find the most likely circle
            Mat(contours[i]).convertTo(pointsf, CV_32F);
            box = fitEllipse(pointsf);
            if (box.size.width < 18 || box.size.width > 300)
                continue;
            circle_edges = approximation.size();
            num_circle++;
        }else
            continue;
    }
    // ROS_INFO("num_circles:%d", num_circle);
    // ROS_INFO("circle_edges:%d",circle_edges);
    // if (num_circle == 0)
    //     ROS_INFO_STREAM("no circle found");
    // else // for debugging
    // {
    //     circle(masked_image, box.center, 5, Scalar(0, 0, 255), -1);
    //     putText(masked_image, "center", box.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    //     Point onCircle;
    //     // onCircle.x = box.center.x;
    //     // onCircle.y = box.center.y + box.size.width;
    //     circle(masked_image, onCircle, 5, Scalar(0, 0, 255), -1);
    //     putText(masked_image, "Point On circle", onCircle, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 8);
    //     imshow("circle", masked_image);
    //     waitKey(1);
    //     cout<<"radius: "<<box.size.width<<endl;
    // }
    // circle(contours_img, box.center, 5, Scalar(0, 0, 255), -1);
    // putText(contours_img, "center", box.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
    // Point onCircle;
    // onCircle.x = box.center.x;
    // onCircle.y = box.center.y + box.size.width / 2;
    // circle(contours_img, onCircle, 5, Scalar(0, 0, 255), -1);
    // imshow("contours", contours_img);
    // imshow("User interface", original_image);
    // waitKey(1);

    //control
    float linear_vel, angular_vel;
    if (!manual_control)
    {   
        double duration;
        float error;
        if (last_tracking_time < 0) // starting auto mode
        {
            // ROS_INFO_STREAM("Started tracking the yellow ball");
            linear_pid.kp = L_P;
            linear_pid.ki = L_I;
            linear_pid.kd = L_D;
            linear_pid.last_error = 0;
            angular_pid.kp = A_P;
            angular_pid.ki = A_I;
            angular_pid.kd = A_D;
            angular_pid.last_error = 0;
            duration = 0.001;
        }else  
            duration = (ros::Time::now().toNSec() - last_tracking_time) / 1000000000;
        // ROS_INFO("Duration:%.8f", duration);
        error = 2 * height / 3 - box.center.y;
        // ROS_INFO("Linear error:%.6f", error);
        linear_vel = pid_cal(linear_pid, error, duration);
        error = width / 2 - box.center.x;
        // ROS_INFO("Angular error:%.6f", error);
        std_msgs::Float32 error_msg;
        error_msg.data = error;
        error_pub.publish(error_msg);
        angular_vel = pid_cal(angular_pid, error, duration);
        // compensation
        if (box.size.width < 0.42 * height) // threshold for amplification is 0.37
        {
            linear_vel *= (1 + COMPENSATION_FACTOR);
            angular_vel *= (1 + COMPENSATION_FACTOR);
        }
        last_tracking_time = ros::Time::now().toNSec();

    }else
    {
        linear_pid.integral_part = 0;
        angular_pid.integral_part = 0;
        last_tracking_time = -1;
        return ;
    }
    
    if (!manual_control) 
    {
        // ROS_INFO("Linear Velocity:%.3f", linear_vel);
        // ROS_INFO("Angular Velocity:%.3f", angular_vel);
        geometry_msgs::Twist tw;
        tw.linear.x = linear_vel;
        tw.angular.z = angular_vel;
        tracking_pub.publish(tw);
    }
}


int main(int argc ,char* *argv) 
{
    ros::init(argc, argv, "yellow_ball_tracking");
    ros::NodeHandle n;
    tracking_sub = n.subscribe("/vrep/image", 1, image_callback);
    error_pub = n.advertise<std_msgs::Float32>("tracking/angular_error", 1);
    tracking_pub = n.advertise<geometry_msgs::Twist>("vrep/cmd_vel", 1);
    msg_sub = n.subscribe("/cmd_vel", 1, message_callback);
    ros::spin();
    return 0;
}
