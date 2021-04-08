//
//  trackbar.cpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 3/25/21.
//

#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"

#include "functions.hpp"



/*class trackbar {
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
};*/

// Constructor
trackbar::trackbar(std::string windowName, std::vector<std::string> sliderName={"B","G","R"}, std::vector<int> maxValueLimit={255,255,255}) : windowName(windowName), sliderName(sliderName), maxValueLimit(maxValueLimit) {
    cv::namedWindow(windowName);
    cv::setMouseCallback(windowName, &trackbar::TrackCallBackFunc, this);
    height = 350;
    length = 500;
    window.create(height, length, CV_8UC3);
    leftTrackPos = 0.275*length;
    rightTrackPos = 0.825*length;
    for (int i=0; i<sliderName.size(); i++) {
        minPosition.push_back(leftTrackPos);
        maxPosition.push_back(rightTrackPos);
        minValueLimit.push_back(0);
        minValue.push_back(0);
        maxValue.push_back(maxValueLimit[i]);
        clickingMin.push_back(false);
        clickingMax.push_back(false);
    }
    drawTrackbar();
}

void trackbar::set_initialvalues(std::vector<int> minValue, std::vector<int> maxValue) {
    this->minValue = minValue;
    this->maxValue = maxValue;
    for (int i=0; i<sliderName.size(); i++) {
        minPosition[i] = ceil(minValue[i]*(rightTrackPos-leftTrackPos)*1.0/maxValueLimit[i]) + leftTrackPos;
        maxPosition[i] = ceil(maxValue[i]*(rightTrackPos-leftTrackPos)*1.0/maxValueLimit[i]) + leftTrackPos;
    }
    if (minValue.size()>2) {
        lowerBound = cv::Scalar(minValue[0], minValue[1], minValue[2]);
        upperBound = cv::Scalar(maxValue[0], maxValue[1], maxValue[2]);
    }
}

// Function which creates an interactive trackbar
void trackbar::drawTrackbar() {
    window.setTo(cv::Scalar(255,255,255)); // create a blank canvas
    
    for (int i=0; i<sliderName.size(); i++) {
        minPosition[i] = minPosition[i] < leftTrackPos ? leftTrackPos : minPosition[i];
        maxPosition[i] = maxPosition[i] > rightTrackPos ? rightTrackPos : maxPosition[i];
        minValue[i] = maxValueLimit[i]*(minPosition[i] - leftTrackPos)/(rightTrackPos-leftTrackPos);
        maxValue[i] = maxValueLimit[i]*(maxPosition[i] - leftTrackPos)/(rightTrackPos-leftTrackPos);
    }
    
    for (int i=0; i<sliderName.size(); i++)         // create lines and text for trackbar
    {
        int trackHeight = height*i/sliderName.size() + height*0.1;
        putText(window, sliderName[i], cv::Point(0.05*length,trackHeight), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,0), 2);
        putText(window, std::to_string(minValue[i]), cv::Point(length*0.175,trackHeight), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255,0,0), 2);
        putText(window, std::to_string(maxValue[i]), cv::Point(length*0.875,trackHeight), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0,0,255), 2);
        line(window, cv::Point(leftTrackPos,trackHeight), cv::Point(rightTrackPos,trackHeight), cv::Scalar(0,0,0), 2);
        if (minPosition[i] < maxPosition[i]) {line(window, cv::Point(minPosition[i],trackHeight), cv::Point(maxPosition[i],trackHeight), cv::Scalar(0,0,0), 4);}
        circle(window, cv::Point(minPosition[i],trackHeight), 8, cv::Scalar(255,0,0), cv::FILLED);
        circle(window, cv::Point(maxPosition[i],trackHeight), 8, cv::Scalar(0,0,255), cv::FILLED);
    }
    
    if (minValue.size()>2) {
        lowerBound = cv::Scalar(minValue[0], minValue[1], minValue[2]);
        upperBound = cv::Scalar(maxValue[0], maxValue[1], maxValue[2]);
    }
}

// Trackbar callback function
void trackbar::TrackCallBackFunc(int event, int x, int y, int flags, void* userdata) {
    
    trackbar *track = (trackbar*) userdata;
    
    if  ( event == cv::EVENT_LBUTTONDOWN ) {
        for (int i=0; i<track->sliderName.size(); i++) {
            int trackHeight = track->height*i/track->sliderName.size() + track->height*0.1;
            if (abs(y-trackHeight) < 15 && abs(x-track->minPosition[i]) < 10) {
                track->minPosition[i] = x;
                track->clickingMin[i] = true;
            }
            else if (abs(y-trackHeight) < 15 && abs(x-track->maxPosition[i]) < 10) {
                track->maxPosition[i] = x;
                track->clickingMax[i] = true;
            }
        }
        track->drawTrackbar();
     }
    
    else if ( event == cv::EVENT_MOUSEMOVE) {
        for (int i=0; i<track->sliderName.size(); i++) {
            if (track->clickingMin[i]) {track->minPosition[i] = x; break;}
            else if (track->clickingMax[i]) {track->maxPosition[i] = x; break;}
        }
        track->drawTrackbar();
    }
    
    else if  ( event == cv::EVENT_LBUTTONUP ) {
        for (int i=0; i<track->sliderName.size(); i++) {
            track->clickingMin[i] = false;
            track->clickingMax[i] = false;
        }
        track->drawTrackbar();
    }
}
