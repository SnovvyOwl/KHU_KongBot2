#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc.hpp>
using namespace std;
int main ( int argc,char **argv ) {
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, 320 );
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 240);
    if (!Camera.open()) {
        cerr<<"Error opening the camera"<<endl;return -1;
    }
    while(1){
        Camera.grab();
        Camera.retrieve ( image);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::imshow( "test", image );
        if ( cv::waitKey(20) == 27 ) break; 
    }
    Camera.release();
}
