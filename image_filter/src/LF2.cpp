#include "LF2.h"
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

Mat FindGreenLine::SobelGrad(Mat &Input_Gray)
{
    // Initializing Variables.
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Mat grad;
    // Applying Gaussian Blur to deal with smaller contours
    GaussianBlur(Input_Gray, Input_Gray, Size(3, 3), 0, 0); //$$ Kernal Size (Default is 3,3) Increasing Makes Blurier. Note: Must be odd number $$
                                                            // Computing Sobel
    //  Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT); [opencv Function Definitions]
    Sobel(Input_Gray, grad_x, CV_16S, 1, 0, 3, 1, 0); //$$ Ksize (3 By Default) $$
    Sobel(Input_Gray, grad_y, CV_16S, 0, 1, 3, 1, 0);
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
    // Thresholding Edges (Weaker Edges Removed)
    threshold(grad, grad, 90, 255, THRESH_BINARY); //$$ Min and Max Threshold (90 and 255 by default) $$
    // Dialating to Fill in a bit:
    dilate(grad, grad, Mat(), Point(-1, -1), 2, 1, 1); //$$ This defaults to 3x3 Kernal for the convolution. Will Need to do more reading if you want more/less $$
    return grad;
}

Mat FindGreenLine::GetSaturation(Mat &Input)
{
    // Intialization.
    cv::Mat hls;
    // Convert Color to HLS Colorspace
    cv::cvtColor(Input, hls, CV_RGB2HLS);
    // Initializing 3 individual single channel images.
    Mat hlsSplit[3];
    // Splitting
    cv::split(hls, hlsSplit);
    // Getting the Saturation Channel
    Mat out = hlsSplit[2];
    // Thresholding to remove lower saturations.
    threshold(out, out, 80, 255, THRESH_BINARY); //$$ Min and Max Threshold (80 and 255 by default) $$
    return out;
}

int FindGreenLine::getMaxAreaContourId(vector<vector<cv::Point>> &contours)
{
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++)
    {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea)
        {
            maxArea = newArea;
            maxAreaContourId = j;
        }
    }
    return maxAreaContourId;
}

void FindGreenLine::printVec(std::vector<int> const &input)
{
    std::cout << "OutputVector:" << std::endl;
    for (int i = 0; i < input.size(); i++)
    {
        std::cout << input.at(i) << std::endl;
    }
}

// void FindGreenLine::drawHist(const vector<int> &data, Mat &image, int BoxWidth, int numBoxes, string label)
// {
//     // Creating Empty Image of Appropriate Height
//     Mat Drawing = Mat::zeros(Size(image.cols, image.rows), CV_8UC3);
//     // Determining the Maximum Value. This is what the histogram is normalized around.
//     int Max = *max_element(data.begin(), data.end());
//     // Intializing Variables.
//     int currX = 0;
//     float heightPer;
//     int height;
//     // Loop that draws Each individual box
//     for (int i = 0; i < numBoxes; i++)
//     {
//         // Determine the Appropriate Height for the Rectangle (Normalized)
//         heightPer = static_cast<float>(data[i]) / static_cast<float>(Max);
//         height = heightPer * image.rows;
//         // Defining and Drawing the Rectangle.
//         cv::Rect rect(currX, image.rows - height, BoxWidth, height);
//         cv::rectangle(Drawing, rect, cv::Scalar(0, 255, 0));
//         // Incramenting for next rectangle
//         currX = currX + BoxWidth;
//     }
// }

int FindGreenLine::findPeakX(const vector<int> &data, int BoxWidth)
{
    int MaxIndex = 0;
    for (int i = 1; i < data.size(); i++)
    {
        if (data[i] > data[MaxIndex])
        {
            MaxIndex = i;
        }
    }
    int PeakX = (MaxIndex + 1) * BoxWidth - (BoxWidth / 2);
    return PeakX;
}

std::vector<int> FindGreenLine::getHisto(Mat &Image, int numBoxes)
{
    // Defining a small rectangle at bottom of the image.
    double ROIHper = 15 / 100;
    int ROIWidth = Image.cols;
    int ROIHeight = 75; //$$ Rows from bottom to be considered ROI $$
    cv::Rect RECTROI(0, Image.rows - ROIHeight, ROIWidth, ROIHeight);
    Mat ROI = Image(RECTROI);
    // Function that Generates Histogram. (Counting nonzero pixels over all X-Coordinates. Resolution set by number of boxes)
    int BoxHeight = ROI.rows;
    int BoxWidth = ROI.cols / numBoxes;
    int currX = 0;
    int numNonZero;
    Mat crop;
    std::vector<int> nonZeroArray;
    for (int i = 0; i < numBoxes; i++)
    {
        cv::Rect currRect(currX, 0, BoxWidth, BoxHeight);
        Mat crop = ROI(currRect);
        numNonZero = countNonZero(crop);
        nonZeroArray.push_back(numNonZero);
        currX = currX + BoxWidth;
    }
    // printVec(nonZeroArray);
    return nonZeroArray;
}

int FindGreenLine::findPeak(Mat &Image, int numBoxes)
{
    std::vector<int> Histo = getHisto(Image, numBoxes);
    //drawHist(Histo, Image, Image.cols / numBoxes, numBoxes, "Histogram");
    int Peak = findPeakX(Histo, Image.cols / numBoxes);
    return Peak;
}

std::vector<cv::Point> FindGreenLine::FindLine(Mat &Inverted, Mat &Original)
{
    // Intializing Variables
    int BoxHeight = 40;
    int BoxWidth = 620;
    int NumBoxes = 1;
    int BoxX = 10;
    int BoxY = Inverted.rows - BoxHeight - 200;
    Mat crop;
    std::vector<cv::Point> Line;

    // Loop That Computes the Line. Intial Bounding box is fed off of Histogram
    for (int i = 0; i < NumBoxes; i++)
    {
        // Find Centroid of largest contour
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat contourOutput = crop.clone();
        cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
        // Defining Center
        cv::Point center;
        // If Contour Exists Find Centroid. If none exist, set to Search Region Center.
        if (contours.size() > 0)
        {
            int MaxContourId = getMaxAreaContourId(contours);
            cv::Moments M = cv::moments(contours[MaxContourId]);
            // center(M.m10/M.m00, M.m01/M.m00);
            center.x = M.m10 / M.m00 + BoxX;
            center.y = BoxY + BoxHeight / 2;
        }
        else
        {
            // center(BoxX+BoxWidth/2,BoxY-BoxHeight/2);
            center.x = BoxX + BoxWidth / 2;
            center.y = BoxY - BoxHeight / 2;
        }

        // Incramenting Search Regions.
        BoxY = BoxY - BoxHeight;
        // Add Contour to vector (Pusback)
        Line.push_back(center);
    }
    return Line;
}

std::vector<cv::Point> FindGreenLine::IdentifyLine(Mat &image)
{
    // Changing Image Size to 640x480 px
    resize(image, image, Size(640, 480), INTER_LINEAR);
    // Converting Image to Gray
    cv::Mat original = image.clone();
    cv::Mat Input = image.clone();
    cv::Mat Input_Gray;
    cvtColor(Input, Input_Gray, COLOR_BGR2GRAY);
    // Getting Sobel
    cv::Mat grad = FindGreenLine::SobelGrad(Input_Gray);
    // Getting Saturation
    cv::Mat sat = FindGreenLine::GetSaturation(Input);
    // Combining the Sobel and Saturation images into a single binary image
    cv::Mat comb;
    bitwise_or(grad, sat, comb);
    // Get X-Coordinate for Bounding box by using histogram
    // int PeakX = FindGreenLine::findPeak(comb, 128);
    // Finding and drawing Lines
    std::vector<cv::Point> Line = FindGreenLine::FindLine(comb, original);
    return Line;
}
