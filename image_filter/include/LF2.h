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

#ifndef LF2_H
#define LF2_H

class FindGreenLine
{
public:
    std::vector<cv::Point> IdentifyLine(Mat &image); // Master function that contains all of the private functions. Accepts image and returns line points
private:
    Mat SobelGrad(Mat &Input_Gray);                                                               // Sobel Operator that computes gradient to find edges
    Mat GetSaturation(Mat &Input);                                                                // Function that Returns the thresholded Saturation Image
    int getMaxAreaContourId(vector<vector<cv::Point>> &contours);                                 // Funtion that determines the ID of the contour with the max area. Returns the index of the maximum area.
    void printVec(std::vector<int> const &input);                                                 // Prints the vector you pass (Used for debugging.)
    int findPeak(Mat &Image, int numBoxes);                                                       // Function that contains all histogram functions Returns most likely x-coordinate of line to intialize.
    std::vector<int> getHisto(Mat &Image, int numBoxes);                                          // Function that determines valuse for histogram.
    // void drawHist(const vector<int> &data, Mat &image, int BoxWidth, int numBoxes, string label); // Function that Draws the histogram
    int findPeakX(const vector<int> &data, int BoxWidth);                                         // Function that determines the maximum value in the histogram.
    std::vector<cv::Point> FindLine(Mat &Inverted, Mat &Original);                     // Function that discritizes the deteted line returns vector of x,y points on line
};
#endif
