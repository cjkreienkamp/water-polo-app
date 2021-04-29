//
//  presentation.cpp
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

Mat pool;

void presentation()
{
    Mat cam;
    string path = "Resources/LindberghPool/video.mp4";                          // VIDEO ONLY
    VideoCapture cap(path);                                                     // VIDEO ONLY
    
    Mat waitbar = Mat(200, 500, CV_8UC3);                                       // SAVE-VIDEO ONLY
    int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));          // SAVE-VIDEO ONLY*
    int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    int frame_count = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    unsigned long frameCounter = 0;
    Size frame_size(frame_width, frame_height);
    int frames_per_second = 10;
    VideoWriter camVideoWriter("Resources/LindberghPool/outputCAM.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),frames_per_second, frame_size, true);
    VideoWriter animVideoWriter("Resources/LindberghPool/outputANIM.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),frames_per_second, Size(500,400), true);
    
    // read in data
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    vector<vector<Point2f>> poolBoundary_vector;
    
    // read in data
    ifstream awayPlayers_ifile("Files/awayPlayers");
    ifstream homePlayers_ifile("Files/homePlayers");
    ifstream ball_ifile("Files/ball");
    ifstream sides_ifile("Files/sides");
    ifstream corners_ifile("Files/corners");
    ifstream poolBoundary_ifile("Files/poolBoundary");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16;
    char c;
    while (awayPlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_ifile >> c >> var1 >> c >> var2 >> c) {ball_vector.push_back(Point2f(var1,var2));}
    while (sides_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {sides_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    while (poolBoundary_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    
    // show the video
    unsigned int iterator = 0;
    while (true) {
        cap.read(cam);
        if (cam.empty()) {break;}
        
        drawCamera_PRESENTATION(cam, iterator, awayPlayers_vector, homePlayers_vector, ball_vector, corners_vector);
        drawAnimated_PRESENTATION(iterator, awayPlayers_vector, homePlayers_vector, ball_vector, corners_vector, poolBoundary_vector);
        iterator++;
        
        camVideoWriter.write(cam);  //write the video frame to the file           // SAVE-VIDEO ONLY
        animVideoWriter.write(pool);                                            // SAVE-VIDEO ONLY
        
        // waitbar for saving a video                                               // SAVE-VIDEO ONLY*
        frameCounter++;
        waitbar.setTo(cv::Scalar(175,175,175));
        rectangle(waitbar,Point(50,80),Point(400,120),Scalar(0,0,0),1); // black outline
        rectangle(waitbar,Point(51,81),Point(399,119),Scalar(255,255,255),FILLED); // white box fill
        double percentFinished = static_cast<double>(frameCounter)/frame_count;
        rectangle(waitbar, Point(51,81), Point(51+(399.0-51.0)*percentFinished,119), Scalar(0,0,255),FILLED); // red box fill
        putText(waitbar, to_string(static_cast<int>(100*percentFinished))+"%", Point(420,110), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),1.5);
        imshow("waitbar",waitbar);
        
        int key = waitKey(1);
        if (key==27) {break;}
    }
    camVideoWriter.release(); //flush and close the video file    // SAVE-VIDEO ONLY
    animVideoWriter.release(); //flush and close the video file    // SAVE-VIDEO ONLY
}

void drawCamera_PRESENTATION(Mat cam, int iterator, vector<vector<Point2f>> awayPlayers_vector, vector<vector<Point2f>> homePlayers_vector, vector<Point2f> ball_vector, vector<vector<Point2f>> corners_vector) {
    
    Scalar awayColor = Scalar(255,255,255);
    Scalar homeColor = Scalar(0,255,0);
    Mat camMaskPositive = Mat::zeros(cam.size(), cam.type());
    
    // circle on each player and ball
    for (int i=0; i<awayPlayers_vector[0].size(); i++) {
        circle(cam, awayPlayers_vector[iterator][i], 5, awayColor, FILLED);
        circle(cam, homePlayers_vector[iterator][i], 5, homeColor, FILLED);
    }
    circle(cam, ball_vector[iterator], 5, Scalar(0,255,255), FILLED);
    
    // lines for boundaries
    for (int i=0; i<corners_vector[0].size(); i++) {
        line(camMaskPositive, corners_vector[iterator][i], corners_vector[iterator][(i+1)%corners_vector[0].size()], Scalar(100,100,100), 3);
    }
    
    // imshow camera
    add(cam, camMaskPositive, cam);
    //imshow("Camera", cam);
}

void drawAnimated_PRESENTATION(int iterator, vector<vector<Point2f>> awayPlayers_vector, vector<vector<Point2f>> homePlayers_vector, vector<Point2f> ball_vector, vector<vector<Point2f>> corners_vector, vector<vector<Point2f>> poolBoundaryCamera) {
    
    //Mat pool;
    Scalar awayColor = Scalar(200,200,200);
    Scalar homeColor = Scalar(0,255,0);
    
    // pool colors
    Scalar poolBlue = Scalar(255,255,204);
    Scalar poolYellow = Scalar(0,255,255);
    Scalar poolRed = Scalar(0,0,255);
    Scalar poolWhite = Scalar(255,255,255);
    Vec3b darkerTint = {50,50,50};
    
    // take pool measurements
    float poolLengthMax = 25;                   // [meters]
    //float poolLengthMin = 22.9;                 // [meters]
    float poolWidthMax = 20;                    // [meters]
    //float poolWidthMin = 13;                    // [meters]
    //float floatingGoalLine2Ropes = 0.30;     // [meters]
    //float floatingGoalLine2Wall = 1.66;     // [meters]
    float goalWidth = 3;
    //float goalHeight = 0.90;
    //float goalHeightShallow = 2.40;
    
    // convert measurements to usable values
    float poolLength = poolLengthMax;
    float poolWidth = poolWidthMax;
    float half = poolLength/2;
    int windowLength = poolLength*20;               // CAN CHANGE THIS
    int windowHeight = poolWidth*20;                // CAN CHANGE THIS
    
    // find the homography that converts the camera plane to the animated pool plane
    vector<Point2f> animatedPoolCorners = {Point2f(0,0),Point2f(windowLength,0),Point2f(windowLength,windowHeight),Point2f(0,windowHeight)};
    Mat H = findHomography(corners_vector[iterator],animatedPoolCorners);
    
    // use the homography to find the pool boundary in the animated pool plane due to the camera field of view limits
    vector<Point> poolBoundaryAnimated;
    for (int i=0; i<poolBoundaryCamera[iterator].size(); i++) {
        if (poolBoundaryCamera[iterator][i] == Point2f(-10,-10)) {continue;}
        else {
            float x = poolBoundaryCamera[iterator][i].x*H.at<double>(0,0) + poolBoundaryCamera[iterator][i].y*H.at<double>(0,1) + H.at<double>(0,2);
            float y = poolBoundaryCamera[iterator][i].x*H.at<double>(1,0) + poolBoundaryCamera[iterator][i].y*H.at<double>(1,1) + H.at<double>(1,2);
            float z = poolBoundaryCamera[iterator][i].x*H.at<double>(2,0) + poolBoundaryCamera[iterator][i].y*H.at<double>(2,1) + H.at<double>(2,2);
            poolBoundaryAnimated.push_back(Point(x/z,y/z));
        }
    }
    
    // use the homography to find the animated positions of the players and ball
    vector<Point2f> awayPlayers_animated, homePlayers_animated;
    Point2f ball_animated;
    for (int i=0; i<awayPlayers_vector[iterator].size(); i++) {
        if (awayPlayers_vector[iterator][i] == Point2f(-10,-10)) {
            awayPlayers_animated.push_back(Point2f(-10,-10));
        } else {
            float awayX = awayPlayers_vector[iterator][i].x*H.at<double>(0,0) + awayPlayers_vector[iterator][i].y*H.at<double>(0,1) + H.at<double>(0,2);
            float awayY = awayPlayers_vector[iterator][i].x*H.at<double>(1,0) + awayPlayers_vector[iterator][i].y*H.at<double>(1,1) + H.at<double>(1,2);
            float awayZ = awayPlayers_vector[iterator][i].x*H.at<double>(2,0) + awayPlayers_vector[iterator][i].y*H.at<double>(2,1) + H.at<double>(2,2);
            awayPlayers_animated.push_back(Point2f(awayX/awayZ,awayY/awayZ));
        }
        if (homePlayers_vector[iterator][i] == Point2f(-10,-10)) {
            homePlayers_animated.push_back(Point2f(-10,-10));
        } else {
            float homeX = homePlayers_vector[iterator][i].x*H.at<double>(0,0) + homePlayers_vector[iterator][i].y*H.at<double>(0,1) + H.at<double>(0,2);
            float homeY = homePlayers_vector[iterator][i].x*H.at<double>(1,0) + homePlayers_vector[iterator][i].y*H.at<double>(1,1) + H.at<double>(1,2);
            float homeZ = homePlayers_vector[iterator][i].x*H.at<double>(2,0) + homePlayers_vector[iterator][i].y*H.at<double>(2,1) + H.at<double>(2,2);
            homePlayers_animated.push_back(Point2f(homeX/homeZ,homeY/homeZ));
        }
    }
    if (ball_vector[iterator] == Point2f(-10,-10)) {
        ball_animated = Point2f(-10,-10);
    } else {
        float ballX = ball_vector[iterator].x*H.at<double>(0,0) + ball_vector[iterator].y*H.at<double>(0,1) + H.at<double>(0,2);
        float ballY = ball_vector[iterator].x*H.at<double>(1,0) + ball_vector[iterator].y*H.at<double>(1,1) + H.at<double>(1,2);
        float ballZ = ball_vector[iterator].x*H.at<double>(2,0) + ball_vector[iterator].y*H.at<double>(2,1) + H.at<double>(2,2);
        ball_animated = Point2f(ballX/ballZ,ballY/ballZ);
    }
    
    // draw the animated pool
    pool.create(windowHeight, windowLength, CV_8UC3);
    pool.setTo(poolBlue);                      // create a blank pool canvas
    line(pool, Point(0,windowHeight/2-windowHeight*goalWidth/poolWidth/2), Point(0,windowHeight/2+windowHeight*goalWidth/poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength,windowHeight/2-windowHeight*goalWidth/poolWidth/2), Point(windowLength,windowHeight/2+windowHeight*goalWidth/poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength*2/poolLength,0), Point(windowLength*2/poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength-windowLength*2/poolLength,0), Point(windowLength-windowLength*2/poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength*6/poolLength,0), Point(windowLength*6/poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength-windowLength*6/poolLength,0), Point(windowLength-windowLength*6/poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength*half/poolLength,0), Point(windowLength*half/poolLength,windowHeight), poolWhite,3);      // half pool

    // darken the area of the pool outside the camera field of view
    Mat poolMaskAnimated1 = Mat(pool.rows, pool.cols, pool.type(), Scalar(50,50,50));
    Mat poolMaskAnimated2 = Mat::zeros(pool.rows, pool.cols, pool.type());
    fillConvexPoly(poolMaskAnimated2, poolBoundaryAnimated, Scalar(50,50,50));
    subtract(poolMaskAnimated1, poolMaskAnimated2, poolMaskAnimated1);
    subtract(pool, poolMaskAnimated1, pool);
    for (int i=0; i<poolBoundaryAnimated.size(); i++) {
        line(pool, Point2f(poolBoundaryAnimated[i].x,poolBoundaryAnimated[i].y), Point2f(poolBoundaryAnimated[(i+1)%poolBoundaryAnimated.size()].x,poolBoundaryAnimated[(i+1)%poolBoundaryAnimated.size()].y), Scalar(255,0,0), 2.5);
    }
    
    // circle on each player and ball
    for (int i=0; i<awayPlayers_animated.size(); i++) {
        if (awayPlayers_animated[i] != Point2f(-10,-10)) {circle(pool, awayPlayers_animated[i], 5, awayColor, FILLED);}
        if (homePlayers_animated[i] != Point2f(-10,-10)) {circle(pool, homePlayers_animated[i], 5, homeColor, FILLED);}
    }
    if (ball_animated != Point2f(-10,-10)) {circle(pool, ball_animated, 5, Scalar(0,255,255), FILLED);}
    
    // imshow animated pool
    //imshow("animated pool", pool);
}
