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
// - make the frame queue more efficient
// - put the ball number (1/1) right next to the home+ away+ so that it can be easily seen how many objects are located on the screen
// - make it automatic so that if a player/ball is at (-10,-10), the frame on either side is a waypoint of either the previous position or (-10,-10)

extern team location;
extern team away;
extern team home;
extern team ball;

// DISTINGUISH GOALIE GLOBAL VARIABLES
bool chooseGoalies = false;
bool switchAway = false;
bool switchHome = false;
bool areYouSure = false;
int playerToSwitchWithGoalie = 0;
vector<vector<Point2f>> awayPlayers_vector_goalie;
vector<vector<Point2f>> homePlayers_vector_goalie;
int itFrame_goalie;

// DATA COLLECTION global variables
vector<Point2f> awayPlayers;
vector<Point2f> homePlayers;
Point2f ballPos;
vector<int> waypoints_vector;       // 0=untouched, 1=interpolated, 2=waypoint
bool accuratePoint;
extern bool T4collect_F4edit;
deque<Mat> previous100frames;
int dequeFrontValue;

extern Point2f* ptrPlayer;
vector<string> possibleInputStates_PLAY;
extern string inputState;
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
bool initializing = true;

vector<Point2f> corners;

vector<Point2f> opticalFlowTrackPointsP;
Mat cam_grayP;
Mat old_grayP;
vector<Point2f> new_opticalFlowTrackPointsP;


void datacollection_players(string path) {
    inputState = "initialize";
    inputPressed = false;
    accuratePoint = false;
    
    // declare variables
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    vector<vector<Point2f>> poolBoundary_vector;
    bool finalRun = false;
    bool T4collect_F4edit = false;      // if true allows for the user to follow a ball/player with the mouse, if false allows for frame navigation
    
    // prepare video for frame-by-frame input with the callback function and read in the first frame
    VideoCapture cap;
    cap.open(path+"video.mp4");
    cap.read(cam);
    previous100frames.push_back(cam.clone()); dequeFrontValue = 0;
    namedWindow("Camera");
    setMouseCallback("Camera", CallBackFunc_DATACOLLECTION_PLAY);
    possibleInputStates_PLAY = {"away+", "home+", "away move", "home move", "ball move", "away-", "home-", "ball-"};
    
    // read in data
    ifstream awayPlayers_ifile(path+"awayPlayers");
    ifstream homePlayers_ifile(path+"homePlayers");
    ifstream ball_ifile(path+"ball");
    ifstream sides_ifile(path+"sides");
    ifstream corners_ifile(path+"corners");
    ifstream poolBoundary_ifile(path+"poolBoundary");
    ifstream waypoints_ifile(path+"waypoints");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16;
    int int1;
    char c;
    while (awayPlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_ifile >> c >> var1 >> c >> var2 >> c) {ball_vector.push_back(Point2f(var1,var2));}
    while (sides_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {sides_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    while (poolBoundary_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (waypoints_ifile >> int1) {waypoints_vector.push_back(int1);}
    
    if (awayPlayers_vector.size() < 100) {cout<<"awayPlayers uh oh"<<endl;}
    if (homePlayers_vector.size() < 100) {cout<<"homePlayers uh oh"<<endl;}
    if (ball_vector.size() < 100) {cout<<"ball uh oh"<<endl;}
    if (waypoints_vector.size() < 100) {cout<<"waypoints uh oh"<<endl;}
    
    // edit data to be the length of the number of frames
    frame_count = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    while (awayPlayers_vector.size() < frame_count) {awayPlayers_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (homePlayers_vector.size() < frame_count) {homePlayers_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (ball_vector.size() < frame_count) {ball_vector.push_back(Point2f(-10,-10));}
    while (sides_vector.size() < frame_count) {sides_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (corners_vector.size() < frame_count) {corners_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (poolBoundary_vector.size() < frame_count) {poolBoundary_vector.push_back({Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10), Point2f(-10,-10)});}
    while (waypoints_vector.size() < frame_count) {waypoints_vector.push_back(0);}
    
    /*
     // iterate through data to interpolate all data between waypoints
    for (int i=0; i<frame_count; i++) {
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
    
    if (inputState == "initialize" && !chooseGoalies) {
        awayPlayers = awayPlayers_vector[0];
        homePlayers = homePlayers_vector[0];
        ballPos = ball_vector[0];
        while (true) {
            drawCamera_DATACOLLECTION_PLAY();
            if (accuratePoint) {waypoints_vector[0] = 2;}
            int key = waitKey(1);
            if (key == '1') {inputState = possibleInputStates_PLAY[0];}
            if (key == '2') {inputState = possibleInputStates_PLAY[1];}
            if (key == '3') {inputState = possibleInputStates_PLAY[2];}
            if (key == '4') {inputState = possibleInputStates_PLAY[3];}
            if (key == '5') {inputState = possibleInputStates_PLAY[4];}
            if (key == '6') {inputState = possibleInputStates_PLAY[5];}
            if (key == '7') {inputState = possibleInputStates_PLAY[6];}
            if (key == '8') {inputState = possibleInputStates_PLAY[7];}
            if (key == 'p') {initializing = false; break;}
        }
        awayPlayers_vector[0] = awayPlayers;
        homePlayers_vector[0] = homePlayers;
        ball_vector[0] = ballPos;
        waypoints_vector[0] = 2;
    }
    
    // DISTINGUISH GOALIE INITIALIZE
    if (chooseGoalies) {
        awayPlayers_vector_goalie = awayPlayers_vector;
        homePlayers_vector_goalie = homePlayers_vector;
        awayPlayers = awayPlayers_vector[0];
        homePlayers = homePlayers_vector[0];
        ballPos = ball_vector[0];
        while (true) {
            drawCamera_DATACOLLECTION_PLAY();
            if (accuratePoint) {waypoints_vector[0] = 2;}
            int key = waitKey(1);
            if (key == '1') {switchAway = true;}
            if (key == '2') {switchHome = true;}
            if (key == 'p') {initializing = false; break;}
        }
        awayPlayers_vector[0] = awayPlayers;
        homePlayers_vector[0] = homePlayers;
        ball_vector[0] = ballPos;
    }
    
    // optical flow initialization
    Mat old_frameP = cam.clone();
    cvtColor(old_frameP, old_grayP, COLOR_BGR2GRAY);
    
    // main while loop to read in the video frames
    itFrame = 0;
    int nextIterator = 1;
    while (true) {
        // read in new frame based on the current iterator and the desired iterator
        if (itFrame < nextIterator) {
            if (nextIterator < dequeFrontValue+previous100frames.size()) {itFrame = nextIterator; cam = previous100frames[itFrame-dequeFrontValue];}
            else {
                itFrame = dequeFrontValue+(int)previous100frames.size()-1;
                while (itFrame < nextIterator) {
                    cap.read(cam);
                    if (cam.empty()) {finalRun = true; break;}
                    itFrame++;
                    previous100frames.push_back(cam.clone());
                    if (previous100frames.size()>101) {previous100frames.pop_front(); dequeFrontValue++;}
                }
            }
        } else if (itFrame > nextIterator) {itFrame = nextIterator; cam = previous100frames[itFrame-dequeFrontValue].clone();}
        
        itFrame_goalie = itFrame; // GOALIE
        
        // update the players and ball to be the values stored in the vectors of the current frame number
        //awayPlayers = awayPlayers_vector[itFrame-1];
        //homePlayers = homePlayers_vector[itFrame-1];
        //ballPos = ball_vector[itFrame-1];
        if (waypoints_vector[itFrame]) {
            awayPlayers = awayPlayers_vector[itFrame];
            homePlayers = homePlayers_vector[itFrame];
            ballPos = ball_vector[itFrame];
        }
        accuratePoint = false;
        
        /*// find the expected positions of the players and the ball with optical flow
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
        
        // check to make sure that each player and the ballPos which are out of the frame are at Point2f(-10,-10)
        for (int i=0; i<awayPlayers.size(); i++) {
            if (awayPlayers[i].x < 0 || awayPlayers[i].x > cam.cols || awayPlayers[i].y < 0 || awayPlayers[i].y > cam.rows) {awayPlayers[i] = Point2f(-10,-10);}
            if (homePlayers[i].x < 0 || homePlayers[i].x > cam.cols || homePlayers[i].y < 0 || homePlayers[i].y > cam.rows) {homePlayers[i] = Point2f(-10,-10);}
        }
        if (ballPos.x < 0 || ballPos.x > cam.cols || ballPos.y < 0 || ballPos.y > cam.rows) {ballPos = Point2f(-10,-10);}*/
        
        // wait for user input to determine the state or to move on to the next frame
        while (!chooseGoalies) {
            drawCamera_DATACOLLECTION_PLAY();
            int key = waitKey(1);
            if (key == 27) {finalRun = true; break;}
            if (key == '1') {inputState = possibleInputStates_PLAY[0];}
            if (key == '2') {inputState = possibleInputStates_PLAY[1];}
            if (key == '3') {inputState = possibleInputStates_PLAY[2];}
            if (key == '4') {inputState = possibleInputStates_PLAY[3];}
            if (key == '5') {inputState = possibleInputStates_PLAY[4];}
            if (key == '6') {inputState = possibleInputStates_PLAY[5];}
            if (key == '7') {inputState = possibleInputStates_PLAY[6];}
            if (key == '8') {inputState = possibleInputStates_PLAY[7];}
            if (T4collect_F4edit) {
                if (key == 'p') {
                    waitKey(1000);
                    key = 0;
                    while (key != 'p') {
                        drawCamera_DATACOLLECTION_PLAY();
                        key = waitKey(1);
                        if (key == 27) {finalRun = true; break;}
                        if (key == '1') {inputState = possibleInputStates_PLAY[0];}
                        if (key == '2') {inputState = possibleInputStates_PLAY[1];}
                        if (key == '3') {inputState = possibleInputStates_PLAY[2];}
                        if (key == '4') {inputState = possibleInputStates_PLAY[3];}
                        if (key == '5') {inputState = possibleInputStates_PLAY[4];}
                        if (key == '6') {inputState = possibleInputStates_PLAY[5];}
                        if (key == '7') {inputState = possibleInputStates_PLAY[6];}
                        if (key == '8') {inputState = possibleInputStates_PLAY[7];}
                    }
                }
                nextIterator++;
                break;
            } else {
                if (accuratePoint) {waypoints_vector[itFrame] = 2;}
                if (key == 'q') {nextIterator = itFrame-100; break;}
                if (key == 'w') {nextIterator = itFrame-10; break;}
                if (key == 'e') {nextIterator = itFrame-1; break;}
                if (key == 'r') {nextIterator = itFrame+1; break;}
                if (key == 't') {nextIterator = itFrame+10; break;}
                if (key == 'y') {nextIterator = itFrame+100; break;}
                if (key == 'u') {nextIterator = itFrame+1000; break;}
                if (key == 'i') {nextIterator = itFrame+59000; break;}
            }
        }
        
        // GOALIE wait for user input to determine the state or to move on to the next frame
        if (chooseGoalies) {
            while (true) {
                awayPlayers_vector = awayPlayers_vector_goalie;
                homePlayers_vector = homePlayers_vector_goalie;
                drawCamera_DATACOLLECTION_PLAY();
                int key = waitKey(1);
                if (key == 27) {finalRun = true; break;}
                if (key == '1') {switchAway = true;}
                if (key == '2') {switchHome = true;}
                if (key == 'q') {nextIterator = itFrame-100; break;}
                if (key == 'w') {nextIterator = itFrame-10; break;}
                if (key == 'e') {nextIterator = itFrame-1; break;}
                if (key == 'r') {nextIterator = itFrame+1; break;}
                if (key == 't') {nextIterator = itFrame+10; break;}
                if (key == 'y') {nextIterator = itFrame+100; break;}
                if (key == 'u') {nextIterator = itFrame+1000; break;}
                if (key == 'i') {nextIterator = itFrame+79000; break;}
            }
        }
        
        /*// update the vectors that store the data
        awayPlayers_vector[itFrame] = awayPlayers;
        homePlayers_vector[itFrame] = homePlayers;
        ball_vector[itFrame] = ballPos;*/
        // update the vectors that store the data if a change is made
        if (accuratePoint && !chooseGoalies) {
            awayPlayers_vector[itFrame] = awayPlayers;
            homePlayers_vector[itFrame] = homePlayers;
            ball_vector[itFrame] = ballPos;
            int waypoint1 = -1; int waypoint2 = -1;
            for (int i=(int)itFrame-1; i>(int)itFrame-501; i--) {
                if (i<1) {waypoint1 = 0; break;}
                else if (waypoints_vector[i] == 2) {waypoint1 = i; break;}
            }
            for (int i=(int)itFrame+1; i<(int)itFrame+501; i++) {
                if (i > frame_count-1) {break;}
                else if (waypoints_vector[i] == 2) {waypoint2 = i; break;}
            }
            if (waypoint1 != -1) {
                interpolateBetweenWaypoints(waypoint1, itFrame, awayPlayers_vector, homePlayers_vector, ball_vector);
                for (int i=waypoint1+1; i<itFrame; i++) {waypoints_vector[i] = 1;}
            }
            if (waypoint2 != -1) {
                interpolateBetweenWaypoints(itFrame, waypoint2, awayPlayers_vector, homePlayers_vector, ball_vector);
                for (int i=itFrame+1; i<waypoint2; i++) {waypoints_vector[i] = 1;}
            }
        }
        
        // prepare for the next run
        if (nextIterator < dequeFrontValue) {nextIterator = dequeFrontValue;}
        else if (nextIterator >= frame_count) {nextIterator = frame_count-1;}
        if (finalRun) {break;}
    }
        
    // save data to files
    ofstream awayPlayers_ofile(path+"awayPlayers");
    ofstream homePlayers_ofile(path+"homePlayers");
    ofstream ball_ofile(path+"ball");
    ostream_iterator<vector<Point2f>> awayPlayers_iterator(awayPlayers_ofile, "\n" );
    ostream_iterator<vector<Point2f>> homePlayers_iterator(homePlayers_ofile, "\n" );
    ostream_iterator<Point2f> ball_iterator(ball_ofile, "\n" );
    copy(awayPlayers_vector.begin( ), awayPlayers_vector.end( ), awayPlayers_iterator);
    copy(homePlayers_vector.begin( ), homePlayers_vector.end( ), homePlayers_iterator);
    copy(ball_vector.begin( ), ball_vector.end( ), ball_iterator);
    if (!chooseGoalies) {
        ofstream waypoints_ofile(path+"waypoints");
        ostream_iterator<int> waypoints_iterator(waypoints_ofile, "\n" );
        copy(waypoints_vector.begin( ), waypoints_vector.end( ), waypoints_iterator);
    }
}







void drawCamera_DATACOLLECTION_PLAY() {
    Mat camMask = cam.clone();
    Mat camMaskPositive = Mat::zeros(cam.size(), cam.type());
    Mat camMaskNegative = Mat::zeros(cam.size(), cam.type());
    
    // circle on each player
    for (int i=0; i<awayPlayers.size(); i++) {
        if (awayPlayers[i] != Point2f(-10,-10)) {
            circle(camMask, awayPlayers[i], 5, away.primaryColor, FILLED);
            circle(camMask, awayPlayers[i], 6, away.secondaryColor, 2);
        }
        if (homePlayers[i] != Point2f(-10,-10)) {
            circle(camMask, homePlayers[i], 5, home.primaryColor, FILLED);
            circle(camMask, homePlayers[i], 6, home.secondaryColor, 2);
        }
    }
    
    // DISTINGUISH THE GOALIES
    circle(camMask, awayPlayers[0], 6, Scalar(0,0,0), 2);
    circle(camMask, homePlayers[0], 6, Scalar(0,0,0), 2);
    for (int i=1; i<awayPlayers.size(); i++) {
        if (awayPlayers[i] != Point2f(-10,-10)) {
            putText(camMask, to_string(i), awayPlayers[i], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2);
        }
        if (homePlayers[i] != Point2f(-10,-10)) {
            putText(camMask, to_string(i), homePlayers[i], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2);
        }
    }
    putText(camMask, "G", awayPlayers[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2);
    putText(camMask, "G", homePlayers[0], FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),2);
    
    // circle on the ball
    if (ballPos != Point2f(-10,-10)) {
        circle(camMask, ballPos, 3, ball.primaryColor, FILLED);
        circle(camMask, ballPos, 4, ball.secondaryColor, 2);
    }
    
    // lines for boundaries
    for (int i=0; i<corners.size(); i++) {
        line(camMaskPositive, corners[i], corners[(i+1)%corners.size()], Scalar(100,100,100), 3);
    }
    
    // bar on top
    Scalar color;
    if (waypoints_vector[itFrame] == 0) {color = Scalar(0,0,155);}
    else if (waypoints_vector[itFrame] == 1) {color = Scalar(0,140,170);}
    else if (waypoints_vector[itFrame] == 2) {color = Scalar(0,155,0);}
    else {color = Scalar(0,0,0);}
    rectangle(camMask, Point(0,0), Point(cam.cols,50), color, FILLED);
    
    // black bar on bottom
    rectangle(camMask, Point(0,cam.rows-50), Point(cam.cols,cam.rows), Scalar(0,0,0), FILLED);
    
    // text on top black bar
    vector<string> possibleFrameStates, possibleFrameKeys;
    if (T4collect_F4edit) {
        possibleFrameStates = {"play/pause", "continue to end"};
        possibleFrameKeys = {"P", "C"};
    } else {
        possibleFrameStates = {"-100", "-10", "-1", "+1", "+10", "+100", "+1000"};
        possibleFrameKeys = {"Q", "W", "E", "R", "T", "Y", "U"};
    }
    if (initializing) {
        putText(camMask, "Instructions: ", Point(cam.cols*2.0/10.0-70, 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
        putText(camMask, "Locate the initial positions of the ball and players. Press 'p' when finished.", Point(cam.cols*3.0/10.0-70,25), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
    } else {
        putText(camMask, "Frame navigation:", Point2f(cam.cols*(static_cast<double>(0)+0.5)/(static_cast<double>(possibleFrameStates.size())+2.5), 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
        for (int i=0; i<possibleFrameStates.size(); i++) {
            putText(camMask, possibleFrameKeys[i]+": "+possibleFrameStates[i], Point2f(cam.cols*(static_cast<double>(i)+2.0)/(static_cast<double>(possibleFrameStates.size())+3.5), 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
        }
    }
    putText(camMask, to_string(itFrame)+"/"+to_string(frame_count), Point2f(cam.cols*(static_cast<double>(possibleFrameStates.size())+0.5)/(static_cast<double>(possibleFrameStates.size())+1.5), 25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
    
    // text on bottom black bar
    for (int i=0; i<possibleInputStates_PLAY.size(); i++) {
        if (inputState == possibleInputStates_PLAY[i]) {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
        putText(camMask, to_string((i+1)%10)+": "+possibleInputStates_PLAY[i], Point2f(cam.cols*(static_cast<double>(2*i+1))/(static_cast<double>(2*possibleInputStates_PLAY.size()+1)), cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
    }
    int awayPlayersCount = 0;
    int homePlayersCount = 0;
    int ballCount = (ballPos == Point2f(-10,-10)) ? 0 : 1;
    for (int i=0; i<awayPlayers.size(); i++) {
        if (awayPlayers[i] != Point2f(-10,-10)) {awayPlayersCount++;}
        if (homePlayers[i] != Point2f(-10,-10)) {homePlayersCount++;}
    }
    if (inputState == "away+") {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
    putText(camMask, "("+to_string(awayPlayersCount)+"/"+to_string(awayPlayers.size())+")", Point(cam.cols*(static_cast<double>(2*0+2))/(static_cast<double>(2*possibleInputStates_PLAY.size()+1))+15, cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
    if (inputState == "home+") {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
    putText(camMask, "("+to_string(homePlayersCount)+"/"+to_string(homePlayers.size())+")", Point(cam.cols*(static_cast<double>(2*1+2))/(static_cast<double>(2*possibleInputStates_PLAY.size()+1))+15, cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
    if (inputState == "ball-") {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
    putText(camMask, "("+to_string(ballCount)+"/1)", Point(cam.cols*(static_cast<double>(2*7+2))/(static_cast<double>(2*possibleInputStates_PLAY.size()+1))+5, cam.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
    
    // display any warnings
    if (WARNING_tooManyPlayers) {putText(camMask, "WARNING: too many players", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_tooManyPlayers++; if(countWARNING_tooManyPlayers>10){WARNING_tooManyPlayers = false; countWARNING_tooManyPlayers = 0;}}
    if (WARNING_noPlayersToMove) {putText(camMask, "WARNING: no players to move", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_noPlayersToMove++; if(countWARNING_noPlayersToMove>10){WARNING_noPlayersToMove = false; countWARNING_noPlayersToMove = 0;}}
    if (WARNING_noPlayersToRemove) {putText(camMask, "WARNING: no players to remove", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_noPlayersToRemove++; if(countWARNING_noPlayersToRemove>10){WARNING_noPlayersToRemove = false; countWARNING_noPlayersToRemove = 0;}}
    
    // GOALIE
    if ((switchAway || switchHome) && chooseGoalies) {
        string switcher;
        if (switchAway) {switcher = "switch away";} else {switcher = "switch home";}
        cout<<"are you sure  "+switcher+"? (y/n)"<<endl;;
        int key = 0;
        while (true) {
            key = waitKey(1);
            if (key == 'n') {areYouSure = false; break;}
            if (key == 'y') {areYouSure = true; cout<<"which number would you like to switch with (1-6)?"<<endl; break;}
        }
        while (areYouSure) {
            key = waitKey(1);
            if (key=='1' || key=='2' || key=='3' || key=='4' || key=='5' || key=='6') {
                playerToSwitchWithGoalie = key-'0';
                cout<<"switched "+to_string(playerToSwitchWithGoalie)+" with the goalie"<<endl;
                break;
            }
        }
        if (areYouSure) {
            if (switchAway) {
                for (int k=itFrame_goalie; k<awayPlayers_vector_goalie.size(); k++) {
                    Point2f pointHolder = awayPlayers_vector_goalie[k][playerToSwitchWithGoalie];
                    awayPlayers_vector_goalie[k][playerToSwitchWithGoalie] = awayPlayers_vector_goalie[k][0];
                    awayPlayers_vector_goalie[k][0] = pointHolder;
                }
            } else if (switchHome) {
                for (int k=itFrame_goalie; k<homePlayers_vector_goalie.size(); k++) {
                    Point2f pointHolder = homePlayers_vector_goalie[k][playerToSwitchWithGoalie];
                    homePlayers_vector_goalie[k][playerToSwitchWithGoalie] = homePlayers_vector_goalie[k][0];
                    homePlayers_vector_goalie[k][0] = pointHolder;
                }
            }
            
            switchAway = false; switchHome = false;
            playerToSwitchWithGoalie = 0;
        }
    }
    
    // imshow camera
    add(camMask, camMaskPositive, camMask);
    subtract(camMask, camMaskNegative, camMask);
    imshow("Camera", camMask);
}

void CallBackFunc_DATACOLLECTION_PLAY( int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && !inputPressed) {
        inputPressed = true;
        
        if (inputState == "away+") {
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
        else if (inputState == "away-") {
            double minDist = INT_MAX;
            for (int i=0; i<awayPlayers.size(); i++) {
                if (awayPlayers[i] != Point2f(-10,-10) && distance(Point2f(x,y),awayPlayers[i]) < minDist) {
                    minDist = distance(Point2f(x,y),awayPlayers[i]);
                    ptrPlayer = &awayPlayers[i];}
            }
            if (minDist == INT_MAX) {WARNING_noPlayersToRemove = true;}
            else {*ptrPlayer = Point2f(-10,-10);}
        }
        else if (inputState == "home+") {
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
        else if (inputState == "home-") {
            double minDist = INT_MAX;
            for (int i=0; i<homePlayers.size(); i++) {
                if (homePlayers[i] != Point2f(-10,-10) && distance(Point2f(x,y),homePlayers[i]) < minDist) {
                    minDist = distance(Point2f(x,y),homePlayers[i]);
                    ptrPlayer = &homePlayers[i];}
            }
            if (minDist == INT_MAX) {WARNING_noPlayersToRemove = true;}
            else {*ptrPlayer = Point2f(-10,-10);}
        }
        else if (inputState == "ball-") {
            if (ballPos == Point2f(-10,-10)) {ballPos = Point2f(x,y); ptrPlayer = &ballPos;}
            else {ballPos = Point2f(-10,-10);}
        }
        else if (inputState == "ball move") {
            ptrPlayer = &ballPos;
            *ptrPlayer = Point2f(x,y);
        }
        drawCamera_DATACOLLECTION_PLAY();
    }
    else if (event == EVENT_MOUSEMOVE && inputPressed) {
        if (inputState == "initialize" || inputState == "away+" || inputState == "away move" || inputState == "home+" || inputState == "home move" || inputState == "ball-" || inputState == "ball move" || inputState == "optical flow") {
            if (ptrPlayer != nullptr) {*ptrPlayer = Point2f(x,y);}
        }
        drawCamera_DATACOLLECTION_PLAY();
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
    
    //cout<<"waypoint 1: "<<way1<<" --> "<<ball_vector[way1]<<"   waypoint 2: "<<way2<<" --> "<<ball_vector[way2]<<endl;
    //cout<<"ballPos interpolator: "<<ball_inter<<endl;
    
    
    for (int interpolator=way1+1; interpolator<way2; interpolator++) {
        for (int i=0; i<awayPlayers_vector[0].size(); i++) {
            awayPlayers_vector[interpolator][i].x = awayPlayers_vector[way1][i].x + (interpolator-way1)*awayPlayers_inter[i].x;
            awayPlayers_vector[interpolator][i].y = awayPlayers_vector[way1][i].y + (interpolator-way1)*awayPlayers_inter[i].y;
            homePlayers_vector[interpolator][i].x = homePlayers_vector[way1][i].x + (interpolator-way1)*homePlayers_inter[i].x;
            homePlayers_vector[interpolator][i].y = homePlayers_vector[way1][i].y + (interpolator-way1)*homePlayers_inter[i].y;
        }
        ball_vector[interpolator].x = ball_vector[way1].x + (interpolator-way1)*ball_inter.x;
        ball_vector[interpolator].y = ball_vector[way1].y + (interpolator-way1)*ball_inter.y;
        //cout<<interpolator<<"  -->  "<<ball_vector[interpolator]<<endl;
    }
}
