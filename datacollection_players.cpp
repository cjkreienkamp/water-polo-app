//
//  datacollection.cpp
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
// - rather than interpolate on screen, interpolate the animated pool positional data
// - ballPos - only, not +/-, and include (0/1)

// DATA COLLECTION global variables
vector<Point2f> awayPlayers;
vector<Point2f> homePlayers;
Point2f ballPos;
vector<bool> waypoints_vector;
bool accuratePoint;

extern Point2f* ptrPlayer;
vector<string> possibleInputStates;
extern string inputState;
extern string frameState;
extern bool inputPressed;

bool WARNING_tooManyPlayers = false;
bool WARNING_noPlayersToMove = false;
bool WARNING_noPlayersToRemove = false;
int countWARNING_tooManyPlayers = 0;
int countWARNING_noPlayersToMove = 0;
int countWARNING_noPlayersToRemove = 0;

extern Mat cam;
extern unsigned int itFrame;
extern int frame_count;

vector<Point2f> corners;

vector<Point2f> opticalFlowTrackPointsP;
Mat cam_grayP;
Mat old_grayP;
vector<Point2f> new_opticalFlowTrackPointsP;


void datacollection_players(string path) {
    //?? PUT BACK IN THE GLOBAL AREA
    inputState = "";
    frameState = "R";
    inputPressed = false;
    accuratePoint = false;
    //??
    
    // declare variables
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    vector<vector<Point2f>> poolBoundary_vector;
    bool finalRun = false;
    
    // prepare video for frame-by-frame input with the callback function and read in the first frame
    VideoCapture cap;
    cap.open(path+"video.mp4");
    cap.read(cam);
    namedWindow("Camera");
    setMouseCallback("Camera", CallBackFunc_DATACOLLECTION_PLAYERS);
    possibleInputStates = {"away +", "home +", "away move", "home move", "ballPos move", "away -", "home -", "ballPos +/-", "optical flow"};
    
    // read in data
    ifstream awayPlayers_ifile(path+"awayPlayers");
    ifstream homePlayers_ifile(path+"homePlayers");
    ifstream ball_ifile(path+"ballPos");
    ifstream sides_ifile(path+"sides");
    ifstream corners_ifile(path+"corners");
    ifstream poolBoundary_ifile(path+"poolBoundary");
    //ifstream waypoints_ifile("Files/waypoints");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16;
    //bool bool1;
    char c;
    while (awayPlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_ifile >> c >> var1 >> c >> var2 >> c) {ball_vector.push_back(Point2f(var1,var2));}
    while (sides_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {sides_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    while (poolBoundary_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    //while (waypoints_ifile >> bool1) {waypoints_vector.push_back(bool1);}
    frame_count = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    
    // edit data to be the length of the number of frames
    while (awayPlayers_vector.size() < frame_count) {awayPlayers_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (homePlayers_vector.size() < frame_count) {homePlayers_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (ball_vector.size() < frame_count) {ball_vector.push_back(Point2f(-10,-10));}
    while (sides_vector.size() < frame_count) {sides_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (corners_vector.size() < frame_count) {corners_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (poolBoundary_vector.size() < frame_count) {poolBoundary_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    //while (waypoints_vector.size() < frame_count) {waypoints_vector.push_back(0);}
    
    // iterate through data to interpolate all data between waypoints
    /*for (int i=0; i<frame_count; i++) {
        int waypoint1, waypoint2 = 0;
        if (waypoints_vector[i] == false) {continue;}
        else {
            waypoint1 = i;
            for (int j=i+1; j<frame_count; j++) {
                if (waypoints_vector[j] == true) {
                    waypoint2 = j;
                    break;
                } else if (j == frame_count-1) {waypoint2 = frame_count-1;}
            }
            interpolateBetweenWaypoints(waypoint1, waypoint2, awayPlayers_vector, homePlayers_vector, ball_vector);
            if (waypoint2 == frame_count-1) {break;}
            i = waypoint2-1;
        }
    }*/
    
    
    Mat old_frameP = cam.clone();
    cvtColor(old_frameP, old_grayP, COLOR_BGR2GRAY);
    
    itFrame = 0;
    int nextIterator = 1;
    
    while (true) {
        // read in new frame based on the current iterator and the desired iterator
        if (itFrame>nextIterator) {cap.open(path+"video.mp4"); cap.read(cam); itFrame = 0;}
        while (itFrame<nextIterator) {cap.read(cam); if (cam.empty()) {break;} itFrame++;}
        
        awayPlayers = awayPlayers_vector[itFrame-1];
        homePlayers = homePlayers_vector[itFrame-1];
        ballPos = ball_vector[itFrame-1];
        accuratePoint = false;
        
        
        
        
        if (opticalFlowTrackPointsP.size() < 5) {goodFeaturesToTrack(old_grayP, opticalFlowTrackPointsP, 100, 0.3, 7, Mat(), 7, false, 0.04);}
        
        // general optical flow data
        cvtColor(cam, cam_grayP, COLOR_BGR2GRAY);
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        
        vector<Point2f> playersBall, new_playersBall;
        playersBall = awayPlayers;
        playersBall.insert(playersBall.end(), homePlayers.begin(), homePlayers.end());
        playersBall.push_back(ballPos);
        calcOpticalFlowPyrLK(old_grayP, cam_grayP, playersBall, new_playersBall, status, err, Size(15,15), 2, criteria);
        awayPlayers.clear(); homePlayers.clear();
        for (int i=0; i<new_playersBall.size(); i++) {
            if (i<awayPlayers_vector[0].size()) {awayPlayers.push_back(new_playersBall[i]);}
            else if (i == new_playersBall.size()-1) {ballPos = new_playersBall[i];}
            else {homePlayers.push_back(new_playersBall[i]);}
        }
        old_grayP = cam_grayP.clone();
        
        /*calcOpticalFlowPyrLK(old_grayP, cam_grayP, opticalFlowTrackPointsP, new_opticalFlowTrackPointsP, status, err, Size(15,15), 2, criteria);
        vector<Point2f> good_old, good_new;
        for(uint i = 0; i < opticalFlowTrackPointsP.size(); i++)
        {
            // select good points
            if(status[i] == 1) {
                good_old.push_back(opticalFlowTrackPointsP[i]);
                good_new.push_back(new_opticalFlowTrackPointsP[i]);
            }
        }
        // update the previous frame and previous points
        opticalFlowTrackPointsP = good_new;
        old_grayP = cam_grayP.clone();

        if (opticalFlowTrackPointsP.size() > 3) {
            
            Mat globalMotionMatrix = cv::videostab::estimateGlobalMotionLeastSquares(good_old, good_new);
            
            // estimate the position of the players and ballPos based on camera movement
            for (int i=0; i<awayPlayers_vector[0].size(); i++) {
                float xAway = awayPlayers_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(0,0) + awayPlayers_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
                float xHome = homePlayers_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(0,0) + homePlayers_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
                float yAway = awayPlayers_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(1,0) + awayPlayers_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
                float yHome = homePlayers_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(1,0) + homePlayers_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
                float zAway = awayPlayers_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(2,0) + awayPlayers_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
                float zHome = homePlayers_vector[itFrame-1][i].x*globalMotionMatrix.at<float>(2,0) + homePlayers_vector[itFrame-1][i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
                awayPlayers_vector[itFrame][i] = Point2f(xAway/zAway,yAway/zAway);
                homePlayers_vector[itFrame][i] = Point2f(xHome/zHome,yHome/zHome);
            }
            float xBall = ball_vector[itFrame-1].x*globalMotionMatrix.at<float>(0,0) + ball_vector[itFrame-1].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
            float yBall = ball_vector[itFrame-1].x*globalMotionMatrix.at<float>(1,0) + ball_vector[itFrame-1].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
            float zBall = ball_vector[itFrame-1].x*globalMotionMatrix.at<float>(2,0) + ball_vector[itFrame-1].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
            ball_vector[itFrame] = Point2f(xBall/zBall,yBall/zBall);
        } else {
            awayPlayers_vector[itFrame] = awayPlayers_vector[itFrame-1];
            homePlayers_vector[itFrame] = homePlayers_vector[itFrame-1];
            ball_vector[itFrame] = ball_vector[itFrame-1];
        }*/
        
        //awayPlayers = awayPlayers_vector[itFrame];
        //homePlayers = homePlayers_vector[itFrame];
        //ballPos = ball_vector[itFrame];
        
        // check to make sure that each player and the ballPos which are out of the frame are at Point2f(-10,-10)
        for (int i=0; i<awayPlayers.size(); i++) {
            if (awayPlayers[i].x < 0 || awayPlayers[i].x > cam.cols || awayPlayers[i].y < 0 || awayPlayers[i].y > cam.rows) {awayPlayers[i] = Point2f(-10,-10);}
            if (homePlayers[i].x < 0 || homePlayers[i].x > cam.cols || homePlayers[i].y < 0 || homePlayers[i].y > cam.rows) {homePlayers[i] = Point2f(-10,-10);}
        }
        if (ballPos.x < 0 || ballPos.x > cam.cols || ballPos.y < 0 || ballPos.y > cam.rows) {ballPos = Point2f(-10,-10);}
        
        // wait for user input to determine the state or to move on to the next frame
        while (true) {
            corners = corners_vector[itFrame];
            drawCamera_DATACOLLECTION();
            int key = waitKey(10);
            if (key == 27) {finalRun = true; break;}
            if (key == 'p' || itFrame == 1) {
                while (true) {
                    drawCamera_DATACOLLECTION();
                    int key2 = waitKey(10);
                    if (key2 == '1') {inputState = possibleInputStates[0];}
                    if (key2 == '2') {inputState = possibleInputStates[1];}
                    if (key2 == '3') {inputState = possibleInputStates[2];}
                    if (key2 == '4') {inputState = possibleInputStates[3];}
                    if (key2 == '5') {inputState = possibleInputStates[4];}
                    if (key2 == '6') {inputState = possibleInputStates[5];}
                    if (key2 == '7') {inputState = possibleInputStates[6];}
                    if (key2 == '8') {inputState = possibleInputStates[7];}
                    if (key2 == '9') {inputState = possibleInputStates[8];}
                    if (key2 == 'c') {break;}
                }
            }
            nextIterator++;
            break;
            if (inputState == "initialize") {continue;}
            if (key == 'q') {nextIterator = itFrame-100; nextIterator  = (nextIterator<0) ? 0 : nextIterator; frameState="Q"; break;}
            if (key == 'w') {nextIterator = itFrame-10; nextIterator  = (nextIterator<0) ? 0 : nextIterator; frameState="W"; break;}
            if (key == 'e') {nextIterator = itFrame-1; nextIterator  = (nextIterator<0) ? 0 : nextIterator; frameState="E"; break;}
            if (key == 'r') {nextIterator = itFrame+1; nextIterator  = (waypoints_vector.size()>nextIterator) ? nextIterator : static_cast<int>(waypoints_vector.size()-1); frameState="R"; break;}
            if (key == 't') {nextIterator = itFrame+10; nextIterator  = (waypoints_vector.size()>nextIterator) ? nextIterator : static_cast<int>(waypoints_vector.size()-1); frameState="T"; break;}
            if (key == 'y') {nextIterator = itFrame+100; nextIterator  = (waypoints_vector.size()>nextIterator) ? nextIterator : static_cast<int>(waypoints_vector.size()-1); frameState="Y"; break;}
            if (key == 'u') {accuratePoint = false; waypoints_vector[itFrame] = false;}
            if (key == '1') {inputState = possibleInputStates[0];}
            if (key == '2') {inputState = possibleInputStates[1];}
            if (key == '3') {inputState = possibleInputStates[2];}
            if (key == '4') {inputState = possibleInputStates[3];}
            if (key == '5') {inputState = possibleInputStates[4];}
            if (key == '6') {inputState = possibleInputStates[5];}
            if (key == '7') {inputState = possibleInputStates[6];}
            if (key == '8') {inputState = possibleInputStates[7];}
        }
        
        // if the frame was edited
        //if (accuratePoint == true) {
            //cout<<"ACCURATE POINT IS TRUE"<<endl;
            // store the current positions if a change was made
            awayPlayers_vector[itFrame] = awayPlayers;
            homePlayers_vector[itFrame] = homePlayers;
            ball_vector[itFrame] = ballPos;
            //waypoints_vector[itFrame] = true;
            
            // find the previous and next waypoints
            /*int previousWaypoint = itFrame-1;
            int nextWaypoint = itFrame+1;
            for (int decrement=itFrame-1; decrement > itFrame-101; decrement--) {
                if (decrement < 0) {previousWaypoint = 0; break;}
                if (waypoints_vector[decrement] == true || decrement==itFrame-100) {previousWaypoint = decrement; break;}
            }
            for (int increment=itFrame+1; increment < itFrame+101; increment++) {
                if (increment >= waypoints_vector.size()) {nextWaypoint = static_cast<int>(waypoints_vector.size()-1); break;}
                if (waypoints_vector[increment] == true || increment==itFrame+100) {nextWaypoint = increment; break;}
            }
            
            // interpolate the values between the current frame and the closest waypoints
            interpolateBetweenWaypoints(previousWaypoint, itFrame, awayPlayers_vector, homePlayers_vector, ball_vector);
            interpolateBetweenWaypoints(itFrame, nextWaypoint, awayPlayers_vector, homePlayers_vector, ball_vector);
            
        }*/
        
        if (finalRun) {break;}
    }
    
    // save data to files
    ofstream awayPlayers_ofile(path+"awayPlayers");
    ofstream homePlayers_ofile(path+"homePlayers");
    ofstream ball_ofile(path+"ballPos");
    //ofstream waypoints_ofile("Files/waypoints");
    ostream_iterator<vector<Point2f>> awayPlayers_iterator(awayPlayers_ofile, "\n" );
    ostream_iterator<vector<Point2f>> homePlayers_iterator(homePlayers_ofile, "\n" );
    ostream_iterator<Point2f> ball_iterator(ball_ofile, "\n" );
    //ostream_iterator<bool> waypoints_iterator(waypoints_ofile, "\n" );
    copy(awayPlayers_vector.begin( ), awayPlayers_vector.end( ), awayPlayers_iterator);
    copy(homePlayers_vector.begin( ), homePlayers_vector.end( ), homePlayers_iterator);
    copy(ball_vector.begin( ), ball_vector.end( ), ball_iterator);
    //copy(waypoints_vector.begin( ), waypoints_vector.end( ), waypoints_iterator);
}





/*double distance(Point2f a, Point2f b) {
    double dist = sqrt(pow(a.x-b.x,2)+pow(a.y-b.y,2));
    return dist;
}*/







void drawCamera_DATACOLLECTION() {
    Scalar awayColor = Scalar(255,255,255);
    Scalar homeColor = Scalar(0,255,0);
    Mat camMask = cam.clone();
    Mat camMaskPositive = Mat::zeros(cam.size(), cam.type());
    Mat camMaskNegative = Mat::zeros(cam.size(), cam.type());
    
    // circle on each player and ballPos
    for (int i=0; i<awayPlayers.size(); i++) {
        circle(camMask, awayPlayers[i], 5, awayColor, FILLED);
        circle(camMask, homePlayers[i], 5, homeColor, FILLED);
    }
    circle(camMask, ballPos, 5, Scalar(0,255,255), FILLED);
    
    // lines for boundaries
    for (int i=0; i<corners.size(); i++) {
        line(camMaskPositive, corners[i], corners[(i+1)%corners.size()], Scalar(100,100,100), 3);
    }
    
    // optical flow track points
    if (inputState == "optical flow") {
        for (int i=0; i<opticalFlowTrackPointsP.size(); i++) {
            circle(camMask, opticalFlowTrackPointsP[i], 5, Scalar(255,105,180), FILLED);
        }
    }
    
    // black bar on top and bottom
    rectangle(camMask, Point(0,0), Point(cam.cols,50), Scalar(0,0,0), FILLED);
    rectangle(camMask, Point(0,cam.rows-50), Point(cam.cols,cam.rows), Scalar(0,0,0), FILLED);
    
    // text on top black bar
    vector<string> possibleFrameStates = {"-100 frames", "-10 frames", "-1 frame", "+1 frame", "+10 frames", "+100 frames", "delete waypoint"};
    vector<string> possibleFrameKeys = {"Q", "W", "E", "R", "T", "Y", "U"};
    for (int i=0; i<possibleFrameStates.size(); i++) {
        if (frameState == possibleFrameStates[i]) {putText(camMask, possibleFrameKeys[i]+": "+possibleFrameStates[i], Point2f(cam.cols*(static_cast<double>(i)+0.5)/(static_cast<double>(possibleFrameStates.size()+1)+1.5), 25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);
        } else {
        putText(camMask, possibleFrameKeys[i]+": "+possibleFrameStates[i], Point2f(cam.cols*(static_cast<double>(i)+0.5)/(static_cast<double>(possibleFrameStates.size()+1)+1.5), 25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
        }
    }
    //if (waypoints_vector[itFrame] == false && accuratePoint == false) {putText(camMask, "INTERPOLATED", Point2f(cam.cols*(static_cast<double>(possibleFrameStates.size())+0.5)/(static_cast<double>(possibleFrameStates.size()+1)+1.5), 25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
    //else {putText(camMask, "WAYPOINT", Point2f(cam.cols*(static_cast<double>(possibleFrameStates.size())+0.5)/(static_cast<double>(possibleFrameStates.size()+1)+1.5), 25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
    putText(camMask, to_string(itFrame)+"/"+to_string(frame_count), Point2f(cam.cols*(static_cast<double>(possibleFrameStates.size()+1)+0.5)/(static_cast<double>(possibleFrameStates.size()+1)+1.5), 25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
    
    // text on bottom black bar
    for (int i=0; i<possibleInputStates.size(); i++) {
        if (inputState == possibleInputStates[i]) {putText(camMask, to_string((i+1)%possibleInputStates.size())+": "+possibleInputStates[i], Point(cam.cols*i/static_cast<double>(possibleInputStates.size())+20, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);
        } else {
        putText(camMask, to_string((i+1)%10)+": "+possibleInputStates[i], Point(cam.cols*i/static_cast<double>(possibleInputStates.size())+20, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
        }
    }
    if (inputState == "optical flow") {putText(camMask, "("+to_string(opticalFlowTrackPointsP.size())+")", Point2f(cam.cols-25, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
    else {putText(camMask, "("+to_string(opticalFlowTrackPointsP.size())+")", Point2f(cam.cols-25, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
    
    int awayPlayersCount = 0;
    int homePlayersCount = 0;
    for (int i=0; i<awayPlayers.size(); i++) {
        if (awayPlayers[i] != Point2f(-10,-10)) {awayPlayersCount++;}
        if (homePlayers[i] != Point2f(-10,-10)) {homePlayersCount++;}
    }
    if (inputState == "away +") {putText(camMask, "("+to_string(awayPlayersCount)+"/"+to_string(awayPlayers.size())+")", Point(cam.cols*1/static_cast<double>(possibleInputStates.size())-70, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
    else {putText(camMask, "("+to_string(awayPlayersCount)+"/"+to_string(awayPlayers.size())+")", Point(cam.cols*1/static_cast<double>(possibleInputStates.size())-70, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
    if (inputState == "home +") {putText(camMask, "("+to_string(homePlayersCount)+"/"+to_string(homePlayers.size())+")", Point(cam.cols*2/static_cast<double>(possibleInputStates.size())-70, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
    else {putText(camMask, "("+to_string(homePlayersCount)+"/"+to_string(homePlayers.size())+")", Point(cam.cols*2/static_cast<double>(possibleInputStates.size())-70, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
    
    // display any warnings
    if (WARNING_tooManyPlayers) {putText(camMask, "WARNING: too many players", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_tooManyPlayers++; if(countWARNING_tooManyPlayers>10){WARNING_tooManyPlayers = false; countWARNING_tooManyPlayers = 0;}}
    if (WARNING_noPlayersToMove) {putText(camMask, "WARNING: no players to move", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_noPlayersToMove++; if(countWARNING_noPlayersToMove>10){WARNING_noPlayersToMove = false; countWARNING_noPlayersToMove = 0;}}
    if (WARNING_noPlayersToRemove) {putText(camMask, "WARNING: no players to remove", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_noPlayersToRemove++; if(countWARNING_noPlayersToRemove>10){WARNING_noPlayersToRemove = false; countWARNING_noPlayersToRemove = 0;}}
    
    // imshow camera
    add(camMask, camMaskPositive, camMask);
    subtract(camMask, camMaskNegative, camMask);
    imshow("Camera", camMask);
}

void CallBackFunc_DATACOLLECTION_PLAYERS( int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && !inputPressed) {
        inputPressed = true;
        
        if (inputState == "initialize") {}
        else if (inputState == "away +") {
            for (int i=0; i<awayPlayers.size(); i++) {
                if (awayPlayers[i] == Point2f(-10,-10)) {awayPlayers[i] = Point2f(x,y); ptrPlayer = &awayPlayers[i]; break;}
                else if (i == awayPlayers.size()-1) {WARNING_tooManyPlayers = true;}
            }
        }
        else if (inputState == "away move") {
            double minDist = INT_MAX;
            for (int i=0; i<awayPlayers.size(); i++) {
                if (awayPlayers[i] != Point2f(-10,-10) && distance(Point2f(x,y),awayPlayers[i]) < minDist) {
                    minDist = distance(Point2f(x,y),awayPlayers[i]);
                    ptrPlayer = &awayPlayers[i];
                }
            }
            if (minDist == INT_MAX) {WARNING_noPlayersToMove = true; ptrPlayer = nullptr;}
            else {*ptrPlayer = Point2f(x,y);}
        }
        else if (inputState == "away -") {
            double minDist = INT_MAX;
            for (int i=0; i<awayPlayers.size(); i++) {
                if (awayPlayers[i] != Point2f(-10,-10) && distance(Point2f(x,y),awayPlayers[i]) < minDist) {
                    minDist = distance(Point2f(x,y),awayPlayers[i]);
                    ptrPlayer = &awayPlayers[i];}
            }
            if (minDist == INT_MAX) {WARNING_noPlayersToRemove = true;}
            else {*ptrPlayer = Point2f(-10,-10);}
        }
        else if (inputState == "home +") {
            for (int i=0; i<homePlayers.size(); i++) {
                if (homePlayers[i] == Point2f(-10,-10)) {homePlayers[i] = Point2f(x,y); ptrPlayer = &homePlayers[i]; break;}
                else if (i == homePlayers.size()-1) {WARNING_tooManyPlayers = true;}
            }
        }
        else if (inputState == "home move") {
            double minDist = INT_MAX;
            for (int i=0; i<homePlayers.size(); i++) {
                if (homePlayers[i] != Point2f(-10,-10) && distance(Point2f(x,y),homePlayers[i]) < minDist) {
                    minDist = distance(Point2f(x,y),homePlayers[i]);
                    ptrPlayer = &homePlayers[i];
                }
            }
            if (minDist == INT_MAX) {WARNING_noPlayersToMove = true; ptrPlayer = nullptr;}
            else {*ptrPlayer = Point2f(x,y);}
        }
        else if (inputState == "home -") {
            double minDist = INT_MAX;
            for (int i=0; i<homePlayers.size(); i++) {
                if (homePlayers[i] != Point2f(-10,-10) && distance(Point2f(x,y),homePlayers[i]) < minDist) {
                    minDist = distance(Point2f(x,y),homePlayers[i]);
                    ptrPlayer = &homePlayers[i];}
            }
            if (minDist == INT_MAX) {WARNING_noPlayersToRemove = true;}
            else {*ptrPlayer = Point2f(-10,-10);}
        }
        else if (inputState == "ballPos +/-") {
            if (ballPos == Point2f(-10,-10)) {ballPos = Point2f(x,y); ptrPlayer = &ballPos;}
            else {ballPos = Point2f(-10,-10);}
        }
        else if (inputState == "ballPos move") {
            ptrPlayer = &ballPos;
            *ptrPlayer = Point2f(x,y);
        }
        else if (inputState == "optical flow") {
            if (opticalFlowTrackPointsP.empty()) {opticalFlowTrackPointsP.push_back(Point2f(x,y)); ptrPlayer = &opticalFlowTrackPointsP[0];}
            else {
                for (int i=0; i<opticalFlowTrackPointsP.size(); i++) {
                    if (distance(opticalFlowTrackPointsP[i], Point2f(x,y)) < 5) {opticalFlowTrackPointsP.erase(opticalFlowTrackPointsP.begin()+i); ptrPlayer = nullptr; break;}
                    else if (i == (opticalFlowTrackPointsP.size()-1)) {opticalFlowTrackPointsP.push_back(Point2f(x,y)); ptrPlayer = &opticalFlowTrackPointsP.back(); break;}
                }
            }
        }
        drawCamera_DATACOLLECTION();
    }
    else if (event == EVENT_MOUSEMOVE && inputPressed) {
        if (inputState == "initialize" || inputState == "away +" || inputState == "away move" || inputState == "home +" || inputState == "home move" || inputState == "ballPos +/-" || inputState == "ballPos move" || inputState == "optical flow") {
            if (ptrPlayer != nullptr) {*ptrPlayer = Point2f(x,y);}
        }
        drawCamera_DATACOLLECTION();
    }
    else if (event == EVENT_LBUTTONUP && inputPressed) {
        inputPressed = false;
        accuratePoint = true;
    }
}

// interpolate between the previous and the next waypoints
void interpolateBetweenWaypoints(int way1, int way2, vector<vector<Point2f>> &awayPlayers_vector, vector<vector<Point2f>> &homePlayers_vector, vector<Point2f> &ball_vector) {
   
    vector<Point2f> awayPlayers_inter, homePlayers_inter;
    
    Point2f ball_inter = Point2f(((ball_vector[way2].x-ball_vector[way1].x)/(way2-way1)), ((ball_vector[way2].y-ball_vector[way1].y)/(way2-way1)));
    for (int i=0; i<awayPlayers_vector[0].size(); i++) {
        float xAway_inter = (awayPlayers_vector[way2][i].x-awayPlayers_vector[way1][i].x)/(way2-way1);
        float yAway_inter = (awayPlayers_vector[way2][i].y-awayPlayers_vector[way1][i].y)/(way2-way1);
        float xHome_inter = (homePlayers_vector[way2][i].x-homePlayers_vector[way1][i].x)/(way2-way1);
        float yHome_inter = (homePlayers_vector[way2][i].y-homePlayers_vector[way1][i].y)/(way2-way1);
        awayPlayers_inter.push_back(Point2f(xAway_inter,yAway_inter));
        homePlayers_inter.push_back(Point2f(xHome_inter,yHome_inter));
    }
    
    cout<<"waypoint 1: "<<way1<<" --> "<<ball_vector[way1]<<"   waypoint 2: "<<way2<<" --> "<<ball_vector[way2]<<endl;
    cout<<"ballPos interpolator: "<<ball_inter<<endl;
    
    
    for (int interpolator=way1+1; interpolator<way2; interpolator++) {
        for (int i=0; i<awayPlayers_vector[0].size(); i++) {
            awayPlayers_vector[interpolator][i].x = awayPlayers_vector[way1][i].x + (interpolator-way1)*awayPlayers_inter[i].x;
            awayPlayers_vector[interpolator][i].y = awayPlayers_vector[way1][i].y + (interpolator-way1)*awayPlayers_inter[i].y;
            homePlayers_vector[interpolator][i].x = homePlayers_vector[way1][i].x + (interpolator-way1)*homePlayers_inter[i].x;
            homePlayers_vector[interpolator][i].y = homePlayers_vector[way1][i].y + (interpolator-way1)*homePlayers_inter[i].y;
        }
        ball_vector[interpolator].x = ball_vector[way1].x + (interpolator-way1)*ball_inter.x;
        ball_vector[interpolator].y = ball_vector[way1].y + (interpolator-way1)*ball_inter.y;
        cout<<interpolator<<"  -->  "<<ball_vector[interpolator]<<endl;
    }
}
