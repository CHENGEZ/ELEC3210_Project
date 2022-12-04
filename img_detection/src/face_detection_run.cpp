#include <ros/ros.h>

#include <opencv2/face.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace cv::face;
using namespace std;

#define PI 3.141592653589793238

ros::Subscriber img_subcriber;
ros::Publisher marker_publisher;
Ptr<face::FaceRecognizer> model;
CascadeClassifier faceDetection;
const string MODEL_PATH = "/home/cyz/catkin_ws/src/img_detection/model.xml";
const string WINDOW_NAME = "Camera Image";

void image_callback(const sensor_msgs::ImageConstPtr &msg)
{
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
        putText(imgcpy, img_person_name + " is detected", Point(10, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(50, 50, 255), 1);
        imshow(WINDOW_NAME, imgcpy);
        waitKey(1);

        // calculate img (x,y) position
        double img_x_pos, img_y_pos;
        img_x_pos = img_y_pos = 0; // hard code just for testing
        /* code*/
        double tl_x = faces[0].tl().x;
        double tl_y = faces[0].tl().y;
        double w = fabs(faces[0].br().x - faces[0].tl().x);
        double h = fabs(faces[0].br().y - faces[0].tl().y);
        img_x_pos = 1 / tan(PI / 8 * h / imgcpy.size().width) * 0.5;
        img_y_pos = (tl_x + w/2) / imgcpy.size().width;
        
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
        putText(imgcpy, "No face is detected", Point(10, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(50, 50, 255), 1);
        imshow(WINDOW_NAME, imgcpy);
        waitKey(1);

        // if there are no faces extracted, clear any markers on screen
        visualization_msgs::Marker marker;
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETE;
        marker_publisher.publish(marker);
    }
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
    img_subcriber = face_detection_node_handle.subscribe("/vrep/image", 1, image_callback);

    ros::spin();

    return 0;
}