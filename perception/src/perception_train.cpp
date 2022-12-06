#include <opencv2/face.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;

Ptr<face::FaceRecognizer> model = face::LBPHFaceRecognizer::create();
const string IMG_NAMES[5] = {"cartoon", "european_man", "green_hair", "obama", "smoker"};
const string PKG_PATH = "/home/cyz/catkin_ws/src/img_detection/";
const string DATASET_PATH = "/home/cyz/catkin_ws/src/img_detection/image_dataset/";

int main(int argc, char **argv)
{
    vector<Mat> images;
    vector<int> labels;
    string img_name;
    Mat img;

    for (int i = 0; i < 5; ++i)
    {
        img_name = IMG_NAMES[i];
        for (int j = 0; j < 5; ++j)
        {
            img = imread(DATASET_PATH + img_name + "_" + to_string(j) + ".png", IMREAD_GRAYSCALE);
            if (img.data == NULL)
            {
                cout << DATASET_PATH + img_name + "_" + to_string(j) << " DNE";
                exit(0);
            }
            images.push_back(img);
            labels.push_back(i);
        }
    }

    cout << "start trainning ..." << endl;
    model->train(images, labels);
    model->save(PKG_PATH + "model.xml");
    cout << "finished trainning, model saved at " << PKG_PATH + "model.xml" << endl;

    return 0;
}