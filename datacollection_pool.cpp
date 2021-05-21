//
//  datacollection_pool.cpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 4/27/21.
//

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

// ### IMPROVEMENTS ###
// - editing mode (same bottom controls, but allow for frame navigation at the top by means of a deque)
// - fix the controls to be move point, translate, collect
// - incorporate the 2M, 6M, and half
// - include a small window to the side which helps for the choosing of proper points

// DATA COLLECTION POOL global variables
vector<Point2f> pointsSide;
vector<Point2f> pointsCorner;
vector<Point2f> poolBoundary;
vector<Point2f> opticalFlowTrackPoints;
//vector<bool> wayptsPool_vector;

Point2f* ptrPlayer;
vector<string> possibleInputStates_POOL;
string inputState = "initialize";
string frameState = "R";
bool inputPressed = false;
bool T4collect_F4edit;
Mat cam;
unsigned int itFrame;
int frame_count;



void datacollection_pool(string path) {
    
    // declare variables
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    vector<vector<Point2f>> poolBoundary_vector;
    bool finalRun = false;
    T4collect_F4edit = true;
    
    // prepare video for frame-by-frame input with the callback function and read in the first frame
    VideoCapture cap;
    cap.open(path+"video.mp4");
    cap.read(cam);
    namedWindow("Camera");
    setMouseCallback("Camera", CallBackFunc_DATACOLLECTION_POOL);
    possibleInputStates_POOL = {"sidelines move", "translate sidelines", "translate all", "collect sidelines", "optical flow"};
    
    if (T4collect_F4edit) {
        // find the sidelines for the first frame
        pointsSide = {Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)};
        while (inputState == "initialize") {drawCamera_DATACOLLECTION_POOL(cam); waitKey(1);}
        sides_vector.push_back(pointsSide);
        corners_vector.push_back(poolSides2Corners(sides_vector[0]));
        poolBoundary_vector.push_back(poolBoundary);
    } else {
        // read in data
        ifstream sides_ifile(path+"sides");
        ifstream corners_ifile(path+"corners");
        ifstream poolBoundary_ifile(path+"poolBoundary");
        double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16;
        char c;
        while (sides_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {sides_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
        while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
        while (poolBoundary_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    }
    
    // edit data to be the length of the number of frames
    frame_count = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    while (sides_vector.size() < frame_count) {sides_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (corners_vector.size() < frame_count) {corners_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (poolBoundary_vector.size() < frame_count) {poolBoundary_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    
    // optical flow initialization
    Mat cam_gray;
    Mat old_gray;
    Mat old_frame = cam.clone();
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    vector<Point2f> new_opticalFlowTrackPoints;
    
    // main while loop to read in the video frames
    itFrame = 0;
    int nextIterator = 1;
    while (true) {
        // read in new frame based on the current iterator and the desired iterator
        if (itFrame>nextIterator) {cap.open(path); cap.read(cam); itFrame = 0;}
        while (itFrame<nextIterator) {cap.read(cam); if (cam.empty()) {break;} itFrame++;}
        
        // general optical flow data
        if (opticalFlowTrackPoints.size() < 5) {goodFeaturesToTrack(old_gray, opticalFlowTrackPoints, 100, 0.3, 7, Mat(), 7, false, 0.04);}
        cvtColor(cam, cam_gray, COLOR_BGR2GRAY);
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, cam_gray, opticalFlowTrackPoints, new_opticalFlowTrackPoints, status, err, Size(15,15), 2, criteria);
        vector<Point2f> good_old, good_new;
        for(uint i = 0; i < opticalFlowTrackPoints.size(); i++)
        {
            // select good points
            if(status[i] == 1) {
                good_old.push_back(opticalFlowTrackPoints[i]);
                good_new.push_back(new_opticalFlowTrackPoints[i]);
            }
        }
        // update the previous frame and previous points
        opticalFlowTrackPoints = good_new;
        old_gray = cam_gray.clone();

        // calculate the new sides based on the optical flow data
        if (opticalFlowTrackPoints.size() > 3) {
            Mat globalMotionMatrix = cv::videostab::estimateGlobalMotionLeastSquares(good_old, good_new);
            // estimate the position of the pool sides based on camera movement
            for (int i=0; i<sides_vector[0].size(); i++) {
                float x = sides_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(0,0) + sides_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
                float y = sides_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(1,0) + sides_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
                float z = sides_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(2,0) + sides_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
                sides_vector[itFrame][i] = Point2f(x/z,y/z);
            }
        } else {sides_vector[itFrame] = sides_vector[itFrame-1];}
        corners_vector[itFrame] = poolSides2Corners(sides_vector[itFrame]);
        poolBoundary_vector[itFrame] = poolBoundary;
        pointsSide = sides_vector[itFrame];
        pointsCorner = poolSides2Corners(pointsSide);
        
        // wait for user input to determine the state or to move on to the next frame
        while (true) {
            drawCamera_DATACOLLECTION_POOL(cam);
            int key = waitKey(1);
            if (key == 27) {finalRun = true; break;}
            if (T4collect_F4edit) {
                if (key == '1') {inputState = possibleInputStates_POOL[0];}
                if (key == '2') {inputState = possibleInputStates_POOL[1];}
                if (key == '3') {inputState = possibleInputStates_POOL[2];}
                if (key == '4') {inputState = possibleInputStates_POOL[3];}
                if (key == '5') {inputState = possibleInputStates_POOL[4];}
                if (key == 'c') {inputState = "continue2end"; break;}
                if (key == 'p') {
                    waitKey(1000);
                    key = 0;
                    while (key != 'p') {
                        drawCamera_DATACOLLECTION_POOL(cam);
                        key = waitKey(1);
                        if (key == 27) {finalRun = true; break;}
                        if (key == 'c') {inputState = "continue2end"; break;}
                        if (key == '1') {inputState = possibleInputStates_POOL[0];}
                        if (key == '2') {inputState = possibleInputStates_POOL[1];}
                        if (key == '3') {inputState = possibleInputStates_POOL[2];}
                        if (key == '4') {inputState = possibleInputStates_POOL[3];}
                        if (key == '5') {inputState = possibleInputStates_POOL[4];}
                    }
                }
                nextIterator++;
                break;
            } else {
                if (key == 'q') {nextIterator = itFrame-100; nextIterator  = (nextIterator<0) ? 0 : nextIterator; frameState="Q"; break;}
                if (key == 'w') {nextIterator = itFrame-10; nextIterator  = (nextIterator<0) ? 0 : nextIterator; frameState="W"; break;}
                if (key == 'e') {nextIterator = itFrame-1; nextIterator  = (nextIterator<0) ? 0 : nextIterator; frameState="E"; break;}
                if (key == 'r') {nextIterator = itFrame+1; nextIterator  = (frame_count>nextIterator) ? nextIterator : static_cast<int>(frame_count-1); frameState="R"; break;}
                if (key == 't') {nextIterator = itFrame+10; nextIterator  = (frame_count>nextIterator) ? nextIterator : static_cast<int>(frame_count-1); frameState="T"; break;}
                if (key == 'y') {nextIterator = itFrame+100; nextIterator  = (frame_count>nextIterator) ? nextIterator : static_cast<int>(frame_count-1); frameState="Y"; break;}
            }
        }
        
        // update the vectors that store the sides data
        sides_vector[itFrame] = pointsSide;
        corners_vector[itFrame] = pointsCorner;
        poolBoundary_vector[itFrame] = poolBoundary;
        
        if (finalRun) {break;}
        if (inputState == "continue2end") {
            for (int i=0; i<frame_count; i++) {
                sides_vector[i] = sides_vector[itFrame];
                corners_vector[i] = corners_vector[itFrame];
                poolBoundary_vector[i] = poolBoundary_vector[itFrame];
            }
            break;
        }
    }
    
    // save data to files
    ofstream sides_ofile(path+"sides");
    ofstream corners_ofile(path+"corners");
    ofstream poolBoundary_ofile(path+"poolBoundary");
    ostream_iterator<vector<Point2f>> sides_iterator(sides_ofile, "\n" );
    ostream_iterator<vector<Point2f>> corners_iterator(corners_ofile, "\n" );
    ostream_iterator<vector<Point2f>> poolBoundary_iterator(poolBoundary_ofile, "\n" );
    copy(sides_vector.begin( ), sides_vector.end( ), sides_iterator);
    copy(corners_vector.begin( ), corners_vector.end( ), corners_iterator);
    copy(poolBoundary_vector.begin( ), poolBoundary_vector.end( ), poolBoundary_iterator);
}





double distance(Point2f a, Point2f b) {
    double dist = sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    return dist;
}

vector<Point2f> poolSides2Corners(vector<Point2f> sides) {
    double m, b, x1, y1, x2, y2;
    vector<vector<double>> mbLines;
    vector<Point2f> corners;
    pointsCorner.clear();
    
    if (sides.size()<2) {
        for (int i=0; i<4; i++) {corners.push_back(Point2f(-10,-10));}
        return corners;
    }
    
    // convert the side points (sides) to slope-intercept lines (mbLines)
    for (int i=0; i<sides.size()/2; i++) {
        x1 = sides[i*2].x; y1 = sides[i*2].y;
        x2 = sides[i*2+1].x; y2 = sides[i*2+1].y;
        m = static_cast<double>(y2-y1)/(x2-x1);
        b = y1 - m*x1;
        mbLines.push_back({m,b});
    }
    
    // create intersection points (corners) from the vector of slope-intercept values (mbLines)
    if (mbLines.size() == 4) {
        for (int i=0; i<mbLines.size(); i++) {
            double x = static_cast<double>((mbLines[(i+1)%4][1] - mbLines[i][1]))/(mbLines[i][0]-mbLines[(i+1)%4][0]);
            double y = mbLines[i][0]*x + mbLines[i][1];
            corners.push_back(Point2f(x,y));
        }
    }
    
    // sort each corner by its angle around the pool centroid (average)
    Point2f poolCentroid = Point2f(0.0, 0.0);
    for (int i=0; i<corners.size(); i++) {
        poolCentroid.x += corners[i].x;
        poolCentroid.y += corners[i].y;
    }
    poolCentroid.x = poolCentroid.x / corners.size();
    poolCentroid.y = poolCentroid.y / corners.size();
    for (int i=0; i<corners.size(); i++) {
        for (int j=static_cast<int>(corners.size()-1); j>i; j--) {
            double centroidAngle1 = atan2(poolCentroid.y-corners[j-1].y, poolCentroid.x-corners[j-1].x);
            double centroidAngle2 = atan2(poolCentroid.y-corners[j].y, poolCentroid.x-corners[j].x);
            if (centroidAngle2 < centroidAngle1) {
                Point2f minValue = corners[j];
                corners[j] = corners[j-1];
                corners[j-1] = minValue;
            }
        }
    }
    if (corners.size()==4) {
        Point2f holder1ForCameraPoolCorners = corners[0];
        Point2f holder2ForCameraPoolCorners = corners[1];
        corners[0] = corners[2];
        corners[1] = corners[3];
        corners[2] = holder1ForCameraPoolCorners;
        corners[3] = holder2ForCameraPoolCorners;
    }
    
    corners2poolBoundary(cam, corners);
    return corners;
}


void corners2poolBoundary(Mat cam, vector<Point2f> corners) {
    
    // find slope-intercept form of each sideline
    double m, b, x1, y1, x2, y2;
    vector<vector<double>> mbLines;
    for (int i=0; i<corners.size(); i++) {
        x1 = corners[i].x; y1 = corners[i].y;
        x2 = corners[(i+1)%corners.size()].x; y2 = corners[(i+1)%corners.size()].y;
        m = static_cast<double>(y2-y1)/(x2-x1);
        b = y1 - m*x1;
        mbLines.push_back({m,b});
    }
    
    // if a corner is outside of the camera, find the point where the sidelines cross the camera border for that corner
    // ??? right now assuming that a corner only exceeds in one dimension and does not cross the corners of the camera boundary
    poolBoundary.clear();
    for (int i=0; i<corners.size(); i++) {
        if (corners[i].x < 0) {
            if (corners[(i-1)%corners.size()].x >= 0) {
                double yBefore = mbLines[(i-1)%mbLines.size()][0] * 0 + mbLines[(i-1)%mbLines.size()][1];
                poolBoundary.push_back(Point2f(0,yBefore));
            }
            if (corners[(i+1)%corners.size()].x >= 0) {
                double yAfter = mbLines[i][0] * 0 + mbLines[i][1];
                poolBoundary.push_back(Point2f(0,yAfter));
            }
        } else if (corners[i].x > cam.cols) {
            if (corners[(i-1)%corners.size()].x <= cam.cols) {
                double yBefore = mbLines[(i-1)%mbLines.size()][0] * cam.cols + mbLines[(i-1)%mbLines.size()][1];
                poolBoundary.push_back(Point2f(cam.cols,yBefore));
            }
            if (corners[(i+1)%corners.size()].x <= cam.cols) {
                double yAfter = mbLines[i][0] * cam.cols + mbLines[i][1];
                poolBoundary.push_back(Point2f(cam.cols,yAfter));
            }
        } else if (corners[i].y < 0) {
            if (corners[(i-1)%corners.size()].y >= 0) {
                double xBefore = (0 - mbLines[(i-1)%mbLines.size()][1]) / mbLines[(i-1)%mbLines.size()][0];
                poolBoundary.push_back(Point2f(xBefore,0));
            }
            if (corners[(i+1)%corners.size()].y >= 0) {
                double xAfter = (0 - mbLines[i][1]) / mbLines[i][0];
                poolBoundary.push_back(Point2f(xAfter,0));
            }
        } else if (corners[i].y > cam.rows) {
            if (corners[(i-1)%corners.size()].y <= cam.rows) {
                double xBefore = (cam.rows - mbLines[(i-1)%mbLines.size()][1]) / mbLines[(i-1)%mbLines.size()][0];
                poolBoundary.push_back(Point2f(xBefore,cam.rows));
            }
            if (corners[(i+1)%corners.size()].y <= cam.rows) {
                double xAfter = (cam.rows - mbLines[i][1]) / mbLines[i][0];
                poolBoundary.push_back(Point2f(xAfter,cam.rows));
            }
        } else {poolBoundary.push_back(corners[i]);}
    }
    
    // continue adding points until poolBoundary has 8 points
    while (poolBoundary.size() < 8) {poolBoundary.push_back(Point2f(-10,-10));}
}





void drawCamera_DATACOLLECTION_POOL(Mat cam) {
    Mat camMask = cam.clone();
    Mat camMaskPositive = Mat::zeros(cam.size(), cam.type());
    Mat camMaskNegative = Mat::zeros(cam.size(), cam.type());
    
    // circles on corners and lines for boundaries
    for (int i=0; i<pointsSide.size(); i++) {circle(camMaskPositive, pointsSide[i], 5, Scalar(100,100,100), FILLED);}
    for (int i=0; i<pointsCorner.size(); i++) {
        circle(camMaskPositive, pointsCorner[i], 8, Scalar(100,100,100), FILLED);
        line(camMaskPositive, pointsCorner[i], pointsCorner[(i+1)%pointsCorner.size()], Scalar(100,100,100), 3);
    }
    
    // optical flow track points
    if (inputState == "optical flow") {
        for (int i=0; i<opticalFlowTrackPoints.size(); i++) {
            circle(camMask, opticalFlowTrackPoints[i], 5, Scalar(255,105,180), FILLED);
        }
    }
    
    // black bar on top and bottom
    rectangle(camMask, Point(0,0), Point(cam.cols,50), Scalar(0,0,0), FILLED);
    rectangle(camMask, Point(0,cam.rows-50), Point(cam.cols,cam.rows), Scalar(0,0,0), FILLED);
    
    // text on top black bar
    vector<string> possibleFrameStates, possibleFrameKeys;
    if (T4collect_F4edit) {
        possibleFrameStates = {"play/pause", "continue to end"};
        possibleFrameKeys = {"P", "C"};
    } else {
        possibleFrameStates = {"-100 frames", "-10 frames", "-1 frame", "+1 frame", "+10 frames", "+100 frames"};
        possibleFrameKeys = {"Q", "W", "E", "R", "T", "Y"};
    }
    if (inputState != "initialize") {
        for (int i=0; i<possibleFrameStates.size(); i++) {
            if (frameState == possibleFrameStates[i]) {putText(camMask, possibleFrameKeys[i]+": "+possibleFrameStates[i], Point2f(cam.cols*(static_cast<double>(i)+0.5)/(static_cast<double>(possibleFrameStates.size())+1.5), 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
            } else {
            putText(camMask, possibleFrameKeys[i]+": "+possibleFrameStates[i], Point2f(cam.cols*(static_cast<double>(i)+0.5)/(static_cast<double>(possibleFrameStates.size())+1.5), 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
            }
        }
    }
    putText(camMask, to_string(itFrame)+"/"+to_string(frame_count), Point2f(cam.cols*(static_cast<double>(possibleFrameStates.size())+0.5)/(static_cast<double>(possibleFrameStates.size())+1.5), 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
    
    // text on bottom black bar
    if (inputState == "initialize" && T4collect_F4edit) {
        int numberSidesRemaining = 0;
        for (int i=0; i<pointsSide.size(); i++) {
            if (pointsSide[i] == Point2f(-10,-10)) {numberSidesRemaining++;}
        }
        if (numberSidesRemaining == 0) {
            inputState = "";
            pointsCorner = poolSides2Corners(pointsSide);
        }
        putText(camMask, "Instructions: ", Point(cam.cols*4.0/10.0-70, cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
        putText(camMask, "Choose "+to_string(numberSidesRemaining)+" remaining sides of pool", Point(cam.cols*5.0/10.0-70,cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
    } else {
        Scalar color;
        for (int i=0; i<possibleInputStates_POOL.size(); i++) {
            if (inputState == possibleInputStates_POOL[i]) {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
            putText(camMask, to_string((i+1)%10)+": "+possibleInputStates_POOL[i], Point2f(cam.cols*(static_cast<double>(2*i+1))/(static_cast<double>(2*possibleInputStates_POOL.size()+1)), cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
        }
        if (inputState == "optical flow") {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
        putText(camMask, "("+to_string(opticalFlowTrackPoints.size())+")", Point2f(cam.cols*(static_cast<double>(possibleInputStates_POOL.size())+0.5)/(static_cast<double>(possibleInputStates_POOL.size()+1)), cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
    }

    // imshow camera
    add(camMask, camMaskPositive, camMask);
    subtract(camMask, camMaskNegative, camMask);
    imshow("Camera", camMask);
}

void CallBackFunc_DATACOLLECTION_POOL( int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && !inputPressed) {
        inputPressed = true;
        
        if (inputState == "initialize") {
            for (int i=0; i<pointsSide.size(); i++) {
                if (pointsSide[i] == Point2f(-10,-10)) {pointsSide[i] = Point2f(x,y); ptrPlayer = &pointsSide[i]; break;}
            }
        }
        else if (inputState == "sidelines move") {
            double minDist = INT_MAX;
            for (int i=0; i<pointsSide.size(); i++) {
                if (pointsSide[i] != Point2f(-10,-10) && distance(Point2f(x,y),pointsSide[i]) < minDist) {
                    minDist = distance(Point2f(x,y),pointsSide[i]);
                    ptrPlayer = &pointsSide[i];
                }
            }
            *ptrPlayer = Point2f(x,y);
            pointsCorner = poolSides2Corners(pointsSide);
        }
        else if (inputState == "optical flow") {
            if (opticalFlowTrackPoints.empty()) {opticalFlowTrackPoints.push_back(Point2f(x,y)); ptrPlayer = &opticalFlowTrackPoints[0];}
            else {
                for (int i=0; i<opticalFlowTrackPoints.size(); i++) {
                    if (distance(opticalFlowTrackPoints[i], Point2f(x,y)) < 5) {opticalFlowTrackPoints.erase(opticalFlowTrackPoints.begin()+i); ptrPlayer = nullptr; break;}
                    else if (i == (opticalFlowTrackPoints.size()-1)) {opticalFlowTrackPoints.push_back(Point2f(x,y)); ptrPlayer = &opticalFlowTrackPoints.back(); break;}
                }
            }
        }
        drawCamera_DATACOLLECTION_POOL(cam);
    }
    
    else if (event == EVENT_MOUSEMOVE && inputPressed) {
        if (inputState == "initialize" || inputState == "optical flow") {
            if (ptrPlayer != nullptr) {*ptrPlayer = Point2f(x,y);}
        } else if (inputState == "sidelines move") {
            *ptrPlayer = Point2f(x,y);
            pointsCorner = poolSides2Corners(pointsSide);
        }
        drawCamera_DATACOLLECTION_POOL(cam);
    }
    
    else if (event == EVENT_LBUTTONUP && inputPressed) {
        inputPressed = false;
        //goodptPool = true;
    }
}


// optical flow between the current and the next waypoint
void opticalflowBetweenWaypoints(int way1, int way2, vector<vector<Point2f>> &sides_vector, vector<vector<Point2f>> &corners_vector, vector<vector<Point2f>> &poolBoundary_vector) {
    
    string path = "data/lindbergh/sluh-lindbergh/040717/video.mp4";
    VideoCapture capOF;
    Mat camOF;
    capOF.open(path);
    capOF.read(camOF);
    int counter = 0;
    while (counter<way1) {capOF.read(camOF); counter++;}
    Mat cam_grayOF;
    Mat old_grayOF;
    Mat old_frameOF = camOF.clone();
    cvtColor(old_frameOF, old_grayOF, COLOR_BGR2GRAY);
    vector<Point2f> new_opticalFlowTrackPoints;
    
    pointsSide = sides_vector[0];
    
    // when entering, the [way1] frame will be showing
    for (int frameIterator=way1+1; frameIterator<way2; frameIterator++) {
        
        if (opticalFlowTrackPoints.size() < 5) {goodFeaturesToTrack(old_grayOF, opticalFlowTrackPoints, 100, 0.3, 7, Mat(), 7, false, 0.04);}
        
        // read in the [frameIterator] frame
        capOF.read(camOF);
        
        // general optical flow data
        cvtColor(camOF, cam_grayOF, COLOR_BGR2GRAY);
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        
        calcOpticalFlowPyrLK(old_grayOF, cam_grayOF, opticalFlowTrackPoints, new_opticalFlowTrackPoints, status, err, Size(15,15), 2, criteria);
        vector<Point2f> good_old, good_new;
        for(uint i = 0; i < opticalFlowTrackPoints.size(); i++)
        {
            // select good points
            if(status[i] == 1) {
                good_old.push_back(opticalFlowTrackPoints[i]);
                good_new.push_back(new_opticalFlowTrackPoints[i]);
            }
        }
        // update the previous frame and previous points
        opticalFlowTrackPoints = good_new;
        old_grayOF = cam_grayOF.clone();

        if (opticalFlowTrackPoints.size() > 3) {
            
            Mat globalMotionMatrix = cv::videostab::estimateGlobalMotionLeastSquares(good_old, good_new);
            
            // estimate the position of the pool sides based on camera movement
            for (int i=0; i<sides_vector[0].size(); i++) {
                float x = sides_vector[frameIterator-1][i].x*globalMotionMatrix.at<float>(0,0) + sides_vector[frameIterator-1][i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
                float y = sides_vector[frameIterator-1][i].x*globalMotionMatrix.at<float>(1,0) + sides_vector[frameIterator-1][i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
                float z = sides_vector[frameIterator-1][i].x*globalMotionMatrix.at<float>(2,0) + sides_vector[frameIterator-1][i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
                sides_vector[frameIterator][i] = Point2f(x/z,y/z);
            }
        } else {sides_vector[frameIterator] = sides_vector[frameIterator-1];}
        
        corners_vector[frameIterator] = poolSides2Corners(sides_vector[frameIterator]);
        poolBoundary_vector[frameIterator] = poolBoundary;
        
        pointsSide = sides_vector[frameIterator];
        pointsCorner = poolSides2Corners(pointsSide);
    }
}
