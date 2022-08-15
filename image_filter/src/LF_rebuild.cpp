#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <vector>
#include <algorithm>
#include <string>
#include <utility>
#include "LF_rebuild.h"

using namespace cv;
using namespace std;

Mat FindBlueLine::SobelGrad(Mat& Input_Gray){ 
    //Initializing Variables.
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Mat grad;
    //Applying Gaussian Blur to deal with smaller contours
    GaussianBlur( Input_Gray, Input_Gray, Size( 3, 3), 0, 0 ); //$$ Kernal Size (Default is 3,3) Increasing Makes Blurier. Note: Must be odd number $$
    //Computing Sobel 
    // Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT); [opencv Function Definitions]
        Sobel(Input_Gray, grad_x, CV_16S, 1, 0, 3, 1, 0);  //$$ Ksize (3 By Default) $$
        Sobel(Input_Gray, grad_y, CV_16S, 0, 1, 3, 1, 0);
        convertScaleAbs(grad_x, abs_grad_x);
        convertScaleAbs(grad_y, abs_grad_y);
        addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    //Thresholding Edges (Weaker Edges Removed)
    threshold(grad, grad, 90, 255, THRESH_BINARY); //$$ Min and Max Threshold (90 and 255 by default) $$
    //Dialating to Fill in a bit: 
    dilate(grad, grad, Mat(), Point(-1, -1), 2, 1, 1); //$$ This defaults to 3x3 Kernal for the convolution. Will Need to do more reading if you want more/less $$
    return grad;
}

Mat FindBlueLine::GetSaturation(Mat& Input){ 
    //Intialization.
    cv::Mat hls;
    //Convert Color to HLS Colorspace
    cv::cvtColor(Input, hls, CV_RGB2HLS);
    //Initializing 3 individual single channel images.
    Mat hlsSplit[3];
    //Splitting
    cv::split(hls, hlsSplit);
    //Getting the Saturation Channel
    Mat out = hlsSplit[2];
    //Thresholding to remove lower saturations.
    threshold(out, out, 80, 255, THRESH_BINARY);//$$ Min and Max Threshold (80 and 255 by default) $$
    return out;
}

int FindBlueLine::findPeakIndex(const vector<int>& data, int BoxWidth){
    int MaxIndex = 0;
    for(int i=1;i<data.size(); i++){
        if(data[i]>data[MaxIndex]){
            MaxIndex = i;
        }
    }
    int PeakX = (MaxIndex+1)*BoxWidth - (BoxWidth/2);
    return PeakX;
}

std::vector<int> FindBlueLine::getLineHisto(Mat& Image,int numBoxes){
    //Function that Generates Histogram. (Counting nonzero pixels over all X-Coordinates. Resolution set by number of boxes)
    int BoxHeight = Image.rows;
    int BoxWidth = Image.cols/numBoxes;
    int currX = 0;
    int numNonZero;
    Mat crop;
    std::vector<int> nonZeroArray;
    for(int i=0; i<numBoxes; i++){
        cv::Rect currRect(currX, 0, BoxWidth, BoxHeight);
        Mat crop = Image(currRect);
        numNonZero = countNonZero(crop);
        nonZeroArray.push_back(numNonZero);
        currX= currX+BoxWidth;
    }
    return nonZeroArray;
}

int FindBlueLine::findPeak2(Mat& Image, int numBoxes){  
    std::vector<int> Histo = getLineHisto(Image, numBoxes);
    int Peak = findPeakIndex(Histo, Image.cols/numBoxes);
    return Peak;
}

std::vector<cv::Point> FindBlueLine::FindLine2(Mat& Inverted) {
    int NumBoxes = 1;
    int BoxHeight = 40;
    int BoxWidth = 620;
    int BoxX = 10;
    int BoxY = Inverted.rows - BoxHeight-200;
    Mat crop;
    std::vector<cv::Point> Line;

    for (int i = 0; i<NumBoxes; i++) {
        cv::Rect currRect(BoxX, BoxY, BoxWidth, BoxHeight);
        Mat crop = Inverted(currRect);
        cv::Point center;
        center.x = FindBlueLine::findPeak2(crop, 64);
        center.y = BoxY;
        Line.push_back(center);

        BoxY = BoxY - BoxHeight;
    }
    return Line;
}

std::vector<cv::Point> FindBlueLine::IdentifyLine(Mat& image){
    //Changing Image Size to 640x480 px
    resize(image, image, Size(640, 480), INTER_LINEAR);
    //Converting Image to Gray
    cv::Mat original = image.clone();
    cv::Mat Input = image.clone();
    cv::Mat Input_Gray ;
    cvtColor(Input, Input_Gray, COLOR_BGR2GRAY);
     //Getting Sobel 
    cv::Mat grad = FindBlueLine::SobelGrad(Input_Gray);
    //Getting Saturation 
    cv::Mat sat = FindBlueLine::GetSaturation(Input);
    //Combining the Sobel and Saturation images into a single binary image
    cv::Mat comb;
    bitwise_or(grad, sat, comb);
    //Finding and drawing Lines
    std::vector<cv::Point> Line = FindBlueLine::FindLine2(comb);
    return Line;
}