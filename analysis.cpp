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

extern team location;

void analysis(string path) {
    VideoCapture cap(path+"video.mp4");                                                     // VIDEO ONLY
    double framesPerSecond = cap.get(CAP_PROP_FPS);
    
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> corners_vector;
    vector<vector<Point2f>> poolBoundary_vector;
    
    vector<vector<Point2f>> awayPlayers_vector_anim;
    vector<vector<Point2f>> homePlayers_vector_anim;
    vector<Point2f> ball_vector_anim;
    vector<vector<Point2f>> poolBoundary_vector_anim;
    deque<Point2f> playerTail_deque;
    vector<vector<Point2f>> playerTail_vector;
    vector<double> awayExposure_vector, homeExposure_vector;
    
    double playerTailSize = (int)(2*log(0.1)/log(0.5)*framesPerSecond);
    while (playerTail_deque.size() < playerTailSize+1) {playerTail_deque.push_back(Point2f(-10,-10));}
    
    // read in data
    ifstream awayPlayers_ifile(path+"awayPlayers");
    ifstream homePlayers_ifile(path+"homePlayers");
    ifstream ball_ifile(path+"ball");
    ifstream corners_ifile(path+"corners");
    ifstream poolBoundary_ifile(path+"poolBoundary");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16;
    char c;
    while (awayPlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_ifile >> c >> var1 >> c >> var2 >> c) {ball_vector.push_back(Point2f(var1,var2));}
    while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    while (poolBoundary_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    
    // take pool measurements
    vector<Point2f> calculatedPoolCorners = {Point2f(0,0),Point2f(location.poolLength,0),Point2f(location.poolLength,location.poolWidth),Point2f(0,location.poolWidth)};
    
    // iterate through each point in time
    cout<<"ANALYSIS: [0........10........20........30........40........50........60........70........80........90.......100]"<<endl<<"          [";       // screen output
    int progress = -1;       // screen output
    for (int i=0; i<awayPlayers_vector.size(); i++) {
        
        // find the homography that converts the camera plane to the animated pool plane
        Mat H = findHomography(corners_vector[i],calculatedPoolCorners);
        
        // use the homography to calculate the positions of the players and store the positions of the players in the animated pool so they do not need to be recalculated in presentation.cpp
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
        awayPlayers_vector_anim.push_back(awayPlayers_calculated);
        homePlayers_vector_anim.push_back(homePlayers_calculated);
        
        // use the homography to find the position of the ball and then store these positions so they do not need to be recalculated in presentation.cpp
        if (ball_vector[i] == Point2f(-10,-10)) {
            ball_vector_anim.push_back(Point2f(-10,-10));
        } else {
            float ballX = ball_vector[i].x*H.at<double>(0,0) + ball_vector[i].y*H.at<double>(0,1) + H.at<double>(0,2);
            float ballY = ball_vector[i].x*H.at<double>(1,0) + ball_vector[i].y*H.at<double>(1,1) + H.at<double>(1,2);
            float ballZ = ball_vector[i].x*H.at<double>(2,0) + ball_vector[i].y*H.at<double>(2,1) + H.at<double>(2,2);
            ball_vector_anim.push_back(Point2f(ballX/ballZ,ballY/ballZ));
        }
        
        // use the homography to find the pool boundary in the animated pool plane due to the camera field of view limits and then store these positions so they do not need to be recalculated in presentation.cpp
        vector<Point2f> poolBoundaryAnimated;
        for (int j=0; j<poolBoundary_vector[0].size(); j++) {
            if (poolBoundary_vector[i][j] == Point2f(-10,-10)) {
                poolBoundaryAnimated.push_back(Point2f(-10,-10));
            } else {
                float x = poolBoundary_vector[i][j].x*H.at<double>(0,0) + poolBoundary_vector[i][j].y*H.at<double>(0,1) + H.at<double>(0,2);
                float y = poolBoundary_vector[i][j].x*H.at<double>(1,0) + poolBoundary_vector[i][j].y*H.at<double>(1,1) + H.at<double>(1,2);
                float z = poolBoundary_vector[i][j].x*H.at<double>(2,0) + poolBoundary_vector[i][j].y*H.at<double>(2,1) + H.at<double>(2,2);
                poolBoundaryAnimated.push_back(Point(x/z,y/z));
            }
        }
        poolBoundary_vector_anim.push_back(poolBoundaryAnimated);
        
        int product = i*100/awayPlayers_vector.size()*86/100;
        if (product > progress) {progress = i*100/awayPlayers_vector.size()*86/100; cout<<product%10;} // screen output
    }
    
    // create a matrix of all of the away and home players of size 14xframe_count for exposure score calculations
    vector<vector<Point2f>> awayhomePlayers_vector;
    for (int i=0; i<awayPlayers_vector[0].size(); i++) {
        vector<Point2f> awayhomePlayers;
        for (int j=0; j<awayPlayers_vector.size(); j++) {
            awayhomePlayers.push_back(awayPlayers_vector_anim[j][i]);
        }
        awayhomePlayers_vector.push_back(awayhomePlayers);
    }
    for (int i=0; i<awayPlayers_vector[0].size(); i++) {
        vector<Point2f> awayhomePlayers;
        for (int j=0; j<awayPlayers_vector.size(); j++) {
            awayhomePlayers.push_back(homePlayers_vector_anim[j][i]);
        }
        awayhomePlayers_vector.push_back(awayhomePlayers);
    }
    
    // set one player as the infected player, then for each time, calculate the exposure score at that instant; add to a running total to find the total exposure score
    vector<double> exposureScore_vector;
    for (int i=0; i<awayhomePlayers_vector.size(); i++) {           // the i_th player is infected
        vector<double> exposureScore = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        for (int j=0; j<awayhomePlayers_vector[0].size(); j++) {    // analyze time j
            
            // create playerTail for the presentation mode
            if (i == 3) {
                // add the current position of the covid-infected player to the (playerTail_deque) queue and remove the front element which was the position of the covid-infected player over 2 seconds ago
                playerTail_deque.pop_front();
                playerTail_deque.push_back(awayhomePlayers_vector[i][j]);
                vector<Point2f> playerTailv;
                for (int k=0; k<playerTail_deque.size(); k++) {
                    playerTailv.push_back(playerTail_deque[k]);
                }
                playerTail_vector.push_back(playerTailv);
                
                // calculate the exposure score for all players at this moment in time for the playerTail
                vector<double> awayExposure, homeExposure;
                for (int k=0; k<awayhomePlayers_vector.size(); k++) {   // exposure score for the k_th player
                    double exposureScoreVid = 0.0;
                    for (int l=j; l>(j-playerTailSize); l--) {          // look at time l for infected player
                        if (l<0) {break;}
                        else if (awayhomePlayers_vector[k][j] == Point2f(-10,-10)) {break;}
                        else if (awayhomePlayers_vector[i][l] == Point2f(-10,-10)) {continue;}
                        else if (distance(awayhomePlayers_vector[k][j],awayhomePlayers_vector[i][l]) <= 1.5) {
                            //cout<<"infected: "<<i<<"   dangerzone: "<<k<<"   time: "<<j<<"   frames before: "<<l<<"   infectedPos: "<<awayhomePlayers_vector[i][l]<<"   dangerPos: "<<awayhomePlayers_vector[k][j]<<endl;
                            double timeBeforeCurrentFrame = (double)(j-l)/framesPerSecond;
                            exposureScoreVid = exp(log(0.5)/2.0*timeBeforeCurrentFrame);
                            break;
                        }
                    }
                    if (k<7) {awayExposure_vector.push_back(exposureScoreVid);}
                    else {homeExposure_vector.push_back(exposureScoreVid);}
                }
            }
            
            for (int k=0; k<awayhomePlayers_vector.size(); k++) {   // exposure score for the k_th player
                for (int l=j; l>(j-playerTailSize); l--) {          // look at time l for infected player
                    if (l<0) {break;}
                    else if (awayhomePlayers_vector[i][l] == Point2f(-10,-10)) {continue;}
                    else if (distance(awayhomePlayers_vector[k][j],awayhomePlayers_vector[i][l]) <= 1.5) {
                        double timeBeforeCurrentFrame = (double)(j-l)/framesPerSecond;
                        exposureScore[k] += exp(log(0.5)/2.0*timeBeforeCurrentFrame);
                        break;
                    }
                }
            }
        }
        
        // push the exposure scores for the current infection of player i to exposureScore_vector; cannot push the entire vector exposureScore because ostream does not correctly output vector<double>
        for (int j=0; j<exposureScore.size(); j++) {
            exposureScore_vector.push_back(exposureScore[j]/framesPerSecond);
        }
        
        int product = 86+i;
        if (product > progress) {progress = 86+i; cout<<product%10;} // screen output
    }
    
    // save data to files
    ofstream awayPlayers_anim_ofile(path+"awayPlayers_anim");
    ofstream homePlayers_anim_ofile(path+"homePlayers_anim");
    ofstream ball_anim_ofile(path+"ball_anim");
    ofstream poolBoundary_anim_ofile(path+"poolBoundary_anim");
    ofstream playerTail_ofile(path+"playerTail");
    ofstream awayExposure_ofile(path+"awayExposure");
    ofstream homeExposure_ofile(path+"homeExposure");
    ofstream exposureScore_ofile(path+"exposureScore");
    ostream_iterator<vector<Point2f>> awayPlayers_anim_iterator(awayPlayers_anim_ofile, "\n" );
    ostream_iterator<vector<Point2f>> homePlayers_anim_iterator(homePlayers_anim_ofile, "\n" );
    ostream_iterator<Point2f> ball_anim_iterator(ball_anim_ofile, "\n" );
    ostream_iterator<vector<Point2f>> poolBoundary_anim_iterator(poolBoundary_anim_ofile, "\n" );
    ostream_iterator<vector<Point2f>> playerTail_iterator(playerTail_ofile, "\n" );
    ostream_iterator<double> awayExposure_iterator(awayExposure_ofile, "\n" );
    ostream_iterator<double> homeExposure_iterator(homeExposure_ofile, "\n" );
    ostream_iterator<double> exposureScore_iterator(exposureScore_ofile, "\n" );
    copy(awayPlayers_vector_anim.begin( ), awayPlayers_vector_anim.end( ), awayPlayers_anim_iterator);
    copy(homePlayers_vector_anim.begin( ), homePlayers_vector_anim.end( ), homePlayers_anim_iterator);
    copy(ball_vector_anim.begin( ), ball_vector_anim.end( ), ball_anim_iterator);
    copy(poolBoundary_vector_anim.begin( ), poolBoundary_vector_anim.end( ), poolBoundary_anim_iterator);
    copy(playerTail_vector.begin( ), playerTail_vector.end( ), playerTail_iterator);
    copy(awayExposure_vector.begin( ), awayExposure_vector.end( ), awayExposure_iterator);
    copy(homeExposure_vector.begin( ), homeExposure_vector.end( ), homeExposure_iterator);
    copy(exposureScore_vector.begin( ), exposureScore_vector.end( ), exposureScore_iterator);
    
    cout<<"0]"<<endl<<endl; // screen output
}
