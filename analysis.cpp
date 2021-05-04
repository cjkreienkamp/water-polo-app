//
//  analysis.cpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 4/29/21.
//

#include <iostream>
#include <fstream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"
#include "opencv2/tracking.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/videostab/global_motion.hpp"
#include <math.h>

#include "functions.hpp"

using namespace std;
using namespace cv;

void analysis(string path) {
    VideoCapture cap(path+"video.mp4");                                                     // VIDEO ONLY
    double framesPerSecond = cap.get(CAP_PROP_FPS);
    
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<vector<Point2f>> corners_vector;
    vector<Point2f> ball_vector;
    deque<Point2f> playerTail;
    vector<vector<Point2f>> playerTail_vector;
    vector<double> awayExposure_vector, homeExposure_vector;
    
    double playerTailSize = 2*log(0.1)/log(0.5)*framesPerSecond+1;
    while (playerTail.size() < playerTailSize) {playerTail.push_back(Point2f(-10,-10));}
    
    // read in data
    ifstream awayPlayers_ifile(path+"awayPlayers");
    ifstream homePlayers_ifile(path+"homePlayers");
    ifstream corners_ifile(path+"corners");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14;
    char c;
    while (awayPlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    
    // take pool measurements
    float poolLengthMax = 25;                   // [meters]
    float poolWidthMax = 20;                    // [meters]

    // convert measurements to usable values
    float poolLength = poolLengthMax;
    float poolWidth = poolWidthMax;
    
    for (int i=0; i<awayPlayers_vector.size(); i++) {
        
        // find the homography that converts the camera plane to the animated pool plane
        vector<Point2f> calculatedPoolCorners = {Point2f(0,0),Point2f(poolLength,0),Point2f(poolLength,poolWidth),Point2f(0,poolWidth)};
        Mat H = findHomography(corners_vector[i],calculatedPoolCorners);
        
        // use the homography to calculate the positions of the players
        vector<Point2f> awayPlayers_calculated, homePlayers_calculated;
        for (int j=0; j<awayPlayers_vector[0].size(); j++) {
            if (awayPlayers_vector[i][j] == Point2f(-10,-10)) {
                awayPlayers_calculated.push_back(Point2f(-10,-10));
            } else {
                float awayX = awayPlayers_vector[i][j].x*H.at<double>(0,0) + awayPlayers_vector[i][j].y*H.at<double>(0,1) + H.at<double>(0,2);
                float awayY = awayPlayers_vector[i][j].x*H.at<double>(1,0) + awayPlayers_vector[i][j].y*H.at<double>(1,1) + H.at<double>(1,2);
                float awayZ = awayPlayers_vector[i][j].x*H.at<double>(2,0) + awayPlayers_vector[i][j].y*H.at<double>(2,1) + H.at<double>(2,2);
                awayPlayers_calculated.push_back(Point2f(awayX/awayZ,awayY/awayZ));
            }
            if (homePlayers_vector[i][j] == Point2f(-10,-10)) {
                homePlayers_calculated.push_back(Point2f(-10,-10));
            } else {
                float homeX = homePlayers_vector[i][j].x*H.at<double>(0,0) + homePlayers_vector[i][j].y*H.at<double>(0,1) + H.at<double>(0,2);
                float homeY = homePlayers_vector[i][j].x*H.at<double>(1,0) + homePlayers_vector[i][j].y*H.at<double>(1,1) + H.at<double>(1,2);
                float homeZ = homePlayers_vector[i][j].x*H.at<double>(2,0) + homePlayers_vector[i][j].y*H.at<double>(2,1) + H.at<double>(2,2);
                homePlayers_calculated.push_back(Point2f(homeX/homeZ,homeY/homeZ));
            }
        }
        
        // add the current position of the covid-infected player to the (playerTail) queue and remove the front element which was the position of the covid-infected player over 2 seconds ago
        playerTail.pop_front();
        playerTail.push_back(awayPlayers_calculated[0]);
        vector<Point2f> playerTailv;
        for (int k=0; k<playerTail.size(); k++) {
            playerTailv.push_back(playerTail[k]);
        }
        playerTail_vector.push_back(playerTailv);
        
        // calculate the exposure score for all players at this moment in time
        vector<Point2f> players_calculated;
        players_calculated = awayPlayers_calculated;
        players_calculated.insert(players_calculated.end(), homePlayers_calculated.begin(), homePlayers_calculated.end());
        vector<double> awayExposure, homeExposure;
        for (int i=0; i<players_calculated.size(); i++) {
            double exposureScore = 0.0;
            for (int j=static_cast<int>(playerTail.size()-1); j>-1; j--) {
                if (distance(players_calculated[i], playerTail[j]) <= 1.5) {
                    double timeBeforeCurrentFrame = static_cast<double>(playerTail.size()-1-j)/framesPerSecond;
                    exposureScore = exp(log(0.5)/2.0*timeBeforeCurrentFrame);
                    break;
                }
            }
            if (i<awayPlayers_calculated.size()) {awayExposure_vector.push_back(exposureScore);}
            else {homeExposure_vector.push_back(exposureScore);}
        }
        //awayExposure_vector.push_back(awayExposure);
        //homeExposure_vector.push_back(homeExposure);
    }
    /*for (int j=0; j<awayExposure_vector.size(); j++) {
        for (int i=0; i<awayExposure_vector[0].size(); i++) {
            cout<<awayExposure_vector[j][i]<<" ";
        }
        cout<<endl;
    }*/
    
    
    // save data to files
    ofstream playerTail_ofile(path+"playerTail");
    ofstream awayExposure_ofile(path+"awayExposure", ios::out | ofstream::binary);
    ofstream homeExposure_ofile(path+"homeExposure");
    ostream_iterator<vector<Point2f>> playerTail_iterator(playerTail_ofile, "\n" );
    ostream_iterator<double> awayExposure_iterator(awayExposure_ofile, "\n" );
    ostream_iterator<double> homeExposure_iterator(homeExposure_ofile, "\n" );
    copy(playerTail_vector.begin( ), playerTail_vector.end( ), playerTail_iterator);
    copy(awayExposure_vector.begin( ), awayExposure_vector.end( ), awayExposure_iterator);
    copy(homeExposure_vector.begin( ), homeExposure_vector.end( ), homeExposure_iterator);
}
