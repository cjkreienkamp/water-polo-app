//
//  main.cpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 1/11/21.
//

//### SPECIALTY COMMENTS ###
// POOL-SPECIFIC: this code is specific only to the pool currently under analysis
// VIDEO ONLY: uncomment to analyze a video, comment out any other comments ending with "ONLY"
// PICTURE ONLY: uncomment to analyze a picture, comment out any other comments ending with "ONLY"
// SAVE VIDEO

//### TASKS ###
// delete unnecessary code in every file
// display the camera and animated pool in one window
// use a queue to store the last 100 frames, and you can only go back to the 100 frames behind the furthest one you already hit



#include <iostream>
#include <fstream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/videostab/global_motion.hpp"

#include "functions.hpp"

using namespace std;
using namespace cv;

// global variables
Point p;
vector<vector<double>> poolLines;
vector<Point2f> lastPoolCorners;
vector<Point2f> p0;
vector<Point2f> temporaryPoolCorners;
vector<Point2f> temporaryPoolCornersHolder;
int buttonPressed = 5;
//VideoCapture cap("Resources/LindberghPool/video.mp4");
Mat poolIdentifierImage;
Mat frame;
vector<Point2f> cornerPoints;
bool state2M;       vector<Point2f> points2M;
bool state5M;       vector<Point2f> points5M;
bool stateHalf;     vector<Point2f> pointsHalf;
bool stateCorner;   //vector<Point2f> pointsCorner;
bool stateSide;     //vector<Point2f> pointsSide;
bool moving = false;
int moverVariable;
bool doneInitializing = false;
vector<Point2f> cornerPt1_vector;
vector<Point2f> cornerPt2_vector;
vector<Point2f> cornerPt3_vector;
vector<Point2f> cornerPt4_vector;
//extern vector<Point2f> pointsSide;
//extern vector<Point2f> pointsCorner;

// function declarations
/*vector<Point2f> getPlayerContours(Mat cam, Mat camHSVPrep);
void getContours(Mat cam, Mat camHSVPrep, Mat dst);
Mat colorTwist(Mat img);
Mat histogramEqualization(Mat bgr_image);
Mat drawAnimatedPool(vector<Point2f> cameraPoolCorners, vector<Point2f> cameraBoundaryPoints);//, vector<Point2f> camPlayerLocations);
void LocationCallBackFunc(int event, int x, int y, int flags, void* userdata);
void OpticalFlowCallBackFunc(int event, int x, int y, int flags, void* userdata);
void UserPoolStateCallBackFunc(int event, int x, int y, int flags, void* userdata);
void UserPoolIdentifierCallBackFunc(int event, int x, int y, int flags, void* userdata);
void findPoolCornersFromUserInput(Mat img);
double distancePoint2Line(Point pt, Point ln1_1, Point ln1_2);
Vec4i drawLongLine(Mat img, Vec4i points);
vector<Point2f> findCamPoolBoundary(Mat img);
vector<Point2f> findCamPoolCorners(Mat img, vector<Point2f> poolBoundary);*/




int main() {
    string path = "data/kirkwood/parkwaywest-kirkwood/042921/";
    teamdata(path);
    //datacollection_pool(path);
    //datacollection_players(path);
    analysis(path);
    presentation(path);
}
