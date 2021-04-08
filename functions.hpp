//
//  trackbar.hpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 3/25/21.
//

#ifndef functions_hpp
#define functions_hpp

#include <stdio.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"



class trackbar {
public:
    cv::Mat mask, window;
    std::string windowName;
    int height, length, leftTrackPos, rightTrackPos;
    std::vector<std::string> sliderName;
    std::vector<int> minPosition, maxPosition, minValue, maxValue, minValueLimit, maxValueLimit;
    std::vector<bool> clickingMin, clickingMax;
    cv::Scalar lowerBound, upperBound;
    
    // Constructor
    trackbar(std::string windowName, std::vector<std::string> sliderName, std::vector<int> maxValueLimit);
    void set_initialvalues(std::vector<int> minValue, std::vector<int> maxValue); // Function which creates an interactive trackbar
    void drawTrackbar(); // Trackbar callback function
    static void TrackCallBackFunc(int event, int x, int y, int flags, void* userdata);
};


    

#endif /* functions_hpp */
