#include <ros/ros.h>

#include <opencv2/face.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace cv::face;
using namespace std;

#define PI 3.141592653589793238

enum RoatDir
{
    CLK_WISE = 0,
    ANTI_CLK_WISE = 1
};
enum AngularDisplacement
{
    RIGHT = 0,
    LEFT = 1
};
enum RoomArea
{
    A = 0,
    B = 1,
    C = 2,
    D = 3
};

// display variables
float linear_spd, angular_spd;
RoomArea roomArea;
string areas[4] = {"A", "B", "C", "D"};

// ros publisher and subcriber
ros::Subscriber img_subcriber, slam_subcriber, laser_subcriber, spd_cmd_subcriber;
ros::Publisher marker_publisher;

// face detection and classification models
Ptr<face::FaceRecognizer> model;
CascadeClassifier faceDetection;

const string MODEL_PATH = "/home/cyz/catkin_ws/src/img_detection/model.xml";
const string WINDOW_NAME = "Camera Image";

// values for marker position calculation
double robot_x = 0, robot_y = 0;
double rotationAngle = 0;
RoatDir rotationDir = CLK_WISE;
AngularDisplacement angularDisplacement = LEFT;

static std::vector<float> ranges(902);

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // judge area
    if (robot_x < 4.9)
    {
        if (robot_y > -3.4)
            roomArea = A;
        else if (robot_y <= -3.4 && robot_y > -6.8)
            roomArea = B;
        else if (robot_y <= -6.8)
            roomArea = C;
    }
    else
    {
        roomArea = D;
    }

    // convert from sensor_msgs::Image to cv::Mat
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    Mat img = ptr->image;
    Mat imgcpy = img;

    // first try to extract out the face
    std::vector<Rect> faces;
    faceDetection.detectMultiScale(img, faces, 1.1, 6);

    // visualize the extracted face in an opencv window
    for (int i = 0; i < faces.size(); ++i)
    {
        rectangle(imgcpy, faces[i].tl(), faces[i].br(), Scalar(50, 50, 255), 3);
    }

    if (faces.size() >= 1)
    {
        // if there is face extracted, try to classify the face
        cvtColor(img, img, COLOR_BGR2GRAY);
        Mat face = img(faces[0]);

        string img_person_name;
        switch (model->predict(face))
        {
        case 0:
            img_person_name = "cartoon";
            break;
        case 1:
            img_person_name = "european_man";
            break;
        case 2:
            img_person_name = "green_hair";
            break;
        case 3:
            img_person_name = "obama";
            break;
        case 4:
            img_person_name = "smoker";
            break;
        default:
            break;
        }

        // show the detection result
        putText(imgcpy, img_person_name + " is detected", Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(50, 50, 255), 1);
        putText(imgcpy, "Linear spd: " + to_string(linear_spd), Point(10, 60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 50, 50), 1);
        putText(imgcpy, "Angular spd: " + to_string(angular_spd), Point(10, 90), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 50, 50), 1);
        putText(imgcpy, "Robot Pos: (" + to_string(robot_x) + "," + to_string(robot_y) + ")", Point(10, 120), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 0), 1);
        putText(imgcpy, "In area " + areas[roomArea], Point(10, 150), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(50, 255, 50), 1);
        imshow(WINDOW_NAME, imgcpy);
        waitKey(1);

        // calculate img (x,y) position
        double img_x_pos = robot_x, img_y_pos = robot_y; // set as robot position just for testing
        /* calculation code*/
        double h = ranges[902 / 2];
        double h_pixle = 256.0 / tan(PI / 8);
        float d_signed = (faces[0].br().x + faces[0].tl().x) / 2.0 - 256;
        angularDisplacement = d_signed > 0 ? RIGHT : LEFT;
        double d = fabs(d_signed);
        double theta = atan(d / h_pixle);
        double distance2img = h / cos(theta);
        int delta_ranges_index = 902 * theta / PI;
        distance2img = angularDisplacement == LEFT ? ranges[902 / 2 + delta_ranges_index] : ranges[902 / 2 - delta_ranges_index];
        if (rotationDir == ANTI_CLK_WISE && 0 <= rotationAngle && rotationAngle < PI / 2) // top right direction
        {
            if (angularDisplacement == LEFT)
            {
                img_x_pos = robot_x + distance2img * sin(rotationAngle - theta);
                img_y_pos = robot_y - distance2img * cos(rotationAngle - theta);
            }
            else if (angularDisplacement == RIGHT)
            {
                img_x_pos = robot_x + distance2img * cos(PI / 2 - rotationAngle - theta);
                img_y_pos = robot_y - distance2img * sin(PI / 2 - rotationAngle - theta);
            }
        }
        else if (rotationDir == ANTI_CLK_WISE && PI / 2 <= rotationAngle && rotationAngle <= PI) // top left direction
        {
            if (angularDisplacement == LEFT)
            {
                img_x_pos = robot_x + distance2img * cos(rotationAngle - PI / 2 - theta);
                img_y_pos = robot_y + distance2img * sin(rotationAngle - PI / 2 - theta);
            }
            else if (angularDisplacement == RIGHT)
            {
                img_x_pos = robot_x + distance2img * cos(rotationAngle - PI / 2 + theta);
                img_y_pos = robot_y + distance2img * sin(rotationAngle - PI / 2 + theta);
            }
        }
        else if (rotationDir == CLK_WISE && 0 <= rotationAngle && rotationAngle < PI / 2) // bottom right direction
        {
            if (angularDisplacement == LEFT)
            {
                img_x_pos = robot_x - distance2img * sin(rotationAngle + theta);
                img_y_pos = robot_y - distance2img * cos(rotationAngle + theta);
            }
            else if (angularDisplacement == RIGHT)
            {
                img_x_pos = robot_x - distance2img * sin(rotationAngle - theta);
                img_y_pos = robot_y - distance2img * cos(rotationAngle - theta);
            }
        }
        else if (rotationDir == CLK_WISE && PI / 2 <= rotationAngle && rotationAngle <= PI) // bottom left direction
        {
            if (angularDisplacement == LEFT)
            {
                img_x_pos = robot_x - distance2img * sin(PI - rotationAngle - theta);
                img_y_pos = robot_y + distance2img * cos(PI - rotationAngle - theta);
            }
            else if (angularDisplacement == RIGHT)
            {
                img_x_pos = robot_x - distance2img * sin(PI - rotationAngle + theta);
                img_y_pos = robot_y + distance2img * cos(PI - rotationAngle + theta);
            }
        }

        // add marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.id = 0;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = img_person_name;
        marker.pose.position.x = img_x_pos;
        marker.pose.position.y = img_y_pos;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f; // color may be changed (currently red)
        marker.color.a = 1.0;  // this must be non-zero
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.lifetime = ros::Duration();

        marker_publisher.publish(marker);
    }
    else
    {
        // show the detection result
        putText(imgcpy, "No face is detected", Point(10, 30), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(50, 50, 255), 1);
        putText(imgcpy, "Linear spd: " + to_string(linear_spd), Point(10, 60), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 50, 50), 1);
        putText(imgcpy, "Angular spd: " + to_string(angular_spd), Point(10, 90), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(255, 50, 50), 1);
        putText(imgcpy, "Robot Pos: (" + to_string(robot_x) + "," + to_string(robot_y) + ")", Point(10, 120), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 0), 1);
        putText(imgcpy, "In area " + areas[roomArea], Point(10, 150), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(50, 255, 50), 1);
        imshow(WINDOW_NAME, imgcpy);
        waitKey(1);

        // if there are no faces extracted, clear any markers on screen
        visualization_msgs::Marker marker;
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_publisher.publish(marker);
    }
}

void slam_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    robot_x = msg.get()->pose.position.x;
    robot_y = msg.get()->pose.position.y;
    rotationAngle = 2 * acos(msg.get()->pose.orientation.w);
    rotationDir = msg.get()->pose.orientation.z > 0 ? ANTI_CLK_WISE : CLK_WISE;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr &msg)
{

    for (int i = 0; i < 902; ++i)
    {
        ranges[i] = msg.get()->ranges[i];
    }
}

void spd_cmd_callback(const geometry_msgs::TwistConstPtr &msg)
{
    linear_spd = msg.get()->linear.x;
    angular_spd = msg.get()->angular.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_detection");
    ros::NodeHandle face_detection_node_handle;

    model = face::LBPHFaceRecognizer::create();
    model->read(MODEL_PATH);

    if (!faceDetection.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"))
    {
        ROS_INFO_STREAM("Failed to load face detector. Check environment setup.");
        exit(-1);
    }

    namedWindow(WINDOW_NAME);

    marker_publisher = face_detection_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    spd_cmd_subcriber = face_detection_node_handle.subscribe("/vrep/cmd_vel", 1, spd_cmd_callback);
    laser_subcriber = face_detection_node_handle.subscribe("/vrep/scan", 1, laser_callback);
    slam_subcriber = face_detection_node_handle.subscribe("/slam_out_pose", 1, slam_callback);
    img_subcriber = face_detection_node_handle.subscribe("/vrep/image", 1, image_callback);

    ros::spin();

    return 0;
}