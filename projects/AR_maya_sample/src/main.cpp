//OpenCV webcam example from http://opencv-srf.blogspot.se/2011/09/capturing-images-videos.html

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/imgproc.hpp"

#include <iostream>
#include <queue>
#include <ctime>
#include <string>

#include "mayaConnection.h"


int main(int argc, char* argv[])
{
    MayaConnection mayaConnection;
    mayaConnection.connect();

    cv::VideoCapture cap(0);
    if (!cap.isOpened()) 
    {
        std::cout << "Cannot open the video cam" << std::endl;
        return -1;
    }
    cv::namedWindow("Webcam", cv::WINDOW_AUTOSIZE);

    bool record = false;

    // Video Loop
    uint counter = 0;
    while (true) 
    {

        //handle Keyboard input
        char key = cv::waitKey(1);
        if (key == 27) 
        {
            std::cout << "Esc key is pressed by user" << std::endl;
            break; 
        } else if (key == 'r') 
        {
            record = !record;
            if(record)
                std::cout << "recording ON" << std::endl;
            else
                std::cout << "recording OFF" << std::endl;
        }

        //record frame
        cv::Mat frame;
        if (!cap.read(frame)) 
        {
            std::cout << "Cannot read a frame from video stream" << std::endl;
            break;
        }
        flip(frame, frame, 1);

        //track markers
        // bool new_data_available = tracker.detectAndShow(frame);
        bool new_data_available = true;
        imshow("Webcam", frame );

        //send to maya
        if (mayaConnection.isConnected() && new_data_available && record) 
        {
                int angle = counter % 360;
                int radius = 10;
                
                std::string cmd = "";

                cmd += "if(objExists(\"locator1\")){";
                cmd += "setAttr locator1.t " + std::to_string(radius * cos(angle)) + " ";
                cmd += std::to_string(radius * sin(angle)) + " 0; }";
                cmd += "\n";
                

                mayaConnection.send(cmd);

                ++counter;
        }
    }
    
    return 0;
}