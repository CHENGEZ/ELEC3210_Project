#include <ros/ros.h>

#include <opencv2/face.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace cv::face;
using namespace std;

ros::Subscriber img_subcriber;
ros::Publisher arrow_marker_publisher;
Ptr<face::FaceRecognizer> model;
CascadeClassifier faceDetection;
const string MODEL_PATH = "/home/cyz/catkin_ws/src/img_detection/model.xml";

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
    // convert from sensor_msgs::Image to cv::Mat
    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    Mat img = ptr->image;
    
    // first try to extract out the face
    std::vector<Rect> faces;
    faceDetection.detectMultiScale(img,faces);

    // if there is face extracted, try to classify the extracted face
    if (faces.size() >= 1)
    {
        cvtColor(img, img, COLOR_BGR2GRAY);
        Mat face = img(faces[0]);
        ROS_INFO_STREAM(faces.size()<< " faces are detected");
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

        ROS_INFO_STREAM(img_person_name << " detected in the input image!");

    }
    else
    {
        // clear any markers on screen
        ROS_INFO_STREAM("No face is detected in the input image.");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_detection");
    ros::NodeHandle face_detection_node_handle;

    model = face::LBPHFaceRecognizer::create();
    model->read(MODEL_PATH);

    if(!faceDetection.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"))
    {
        ROS_INFO_STREAM("Failed to load face detector. Check environment setup.");
        exit(-1);
    }

    arrow_marker_publisher = face_detection_node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    img_subcriber = face_detection_node_handle.subscribe("/vrep/image", 1, image_callback);

    ros::spin();

    return 0;
}