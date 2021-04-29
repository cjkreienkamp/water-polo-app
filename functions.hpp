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

// DATA COLLECTION POOL function declarations
void datacollection_pool();
void drawCamera_DATACOLLECTION_POOL(cv::Mat cam);
void CallBackFunc_DATACOLLECTION_POOL( int event, int x, int y, int flags, void* userdata);
void opticalflowBetweenWaypoints(int way1, int way2, std::vector<std::vector<cv::Point2f>> &sides_vector, std::vector<std::vector<cv::Point2f>> &corners_vector, std::vector<std::vector<cv::Point2f>> &poolBoundary_vector);
std::vector<cv::Point2f> poolSides2Corners(std::vector<cv::Point2f> sides);
void corners2poolBoundary(cv::Mat cam, std::vector<cv::Point2f> corners);

double distance(cv::Point2f a, cv::Point2f b);

// DATA COLLECTION PLAYERS function declarations
void datacollection_players();
//double distance(cv::Point2f a, cv::Point2f b);
void drawCamera_DATACOLLECTION();
void CallBackFunc_DATACOLLECTION_PLAYERS( int event, int x, int y, int flags, void* userdata);
void interpolateBetweenWaypoints(int way1, int way2, std::vector<std::vector<cv::Point2f>> &awayPlayers_vector, std::vector<std::vector<cv::Point2f>> &homePlayers_vector, std::vector<cv::Point2f> &ball_vector);

// PRESENTATION function declarations
void presentation();
void drawCamera_PRESENTATION(cv::Mat cam, int iterator, std::vector<std::vector<cv::Point2f>> awayPlayers_vector, std::vector<std::vector<cv::Point2f>> homePlayers_vector,std::vector<cv::Point2f> ball_vector, std::vector<std::vector<cv::Point2f>> corners_vector);
void drawAnimated_PRESENTATION(int iterator, std::vector<std::vector<cv::Point2f>> awayPlayers_vector, std::vector<std::vector<cv::Point2f>> homePlayers_vector,std::vector<cv::Point2f> ball_vector, std::vector<std::vector<cv::Point2f>> corners_vector, std::vector<std::vector<cv::Point2f>> poolBoundaryCamera);

#endif /* functions_hpp */
