#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <vector>
#include <algorithm>
#include <string>
#include <utility>

using namespace cv;
using namespace std;

#ifndef LF_REBUILD_H
#define LF_REBUILD_H


class FindBlueLine{
    public:
        std::vector<cv::Point> IdentifyLine(Mat& image);                                                
    private:
        Mat SobelGrad(Mat& Input_Gray);                                                                 
        Mat GetSaturation(Mat& Input);                                  
        int findPeak2(Mat& Image, int numBoxes); 
            std::vector<int> getLineHisto(Mat& Image,int numBoxes);                                          
            int  findPeakIndex(const vector<int>& data, int BoxWidth);   
        std::vector<cv::Point> FindLine2(Mat& Inverted);   
};

#endif