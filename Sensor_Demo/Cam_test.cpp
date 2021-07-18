#include <iostream>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
using namespace std;
int main ( int argc,char **argv ) {
    raspicam::RaspiCam_Cv Camera;
    cv::Mat image;
    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3);
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, 320 );
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 240);
    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
    //파일로 동영상을 저장하기 위한 준비
    cv::VideoWriter outputVideo;
    cv::Size frameSize(320,240);
    int fps = 25;
    outputVideo.open("output.avi", cv::VideoWriter::fourcc('X','V','I','D'),fps, frameSize, true);
    if (!outputVideo.isOpened())
    {
        cout  << "Could not open the output video for write: " <<"output.avi" << endl;
        return -1;
    }
    while(1){
        Camera.grab();
        Camera.retrieve ( image);
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        cv::imshow( "test", image );
        outputVideo.write(image);
        //아무키나 누르면 종료
        if ( cv::waitKey(1) > 0 ) break;
    }
    Camera.release();
}
//[출처] Raspberry Pi Camera Module를 OpenCV에서 사용하기|작성자 쿠핑
