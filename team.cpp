//
//  teamdata.cpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 4/30/21.
//

#include <iostream>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"
#include <regex>

#include "functions.hpp"

using namespace std;
using namespace cv;

// find out how to parse to find the location, home team, and away team
// class with data on team primary color, team secondary color, team number color
// class with data on location pool length, location pool width

/*class team {
 public:
     cv::Scalar primaryColor;
     cv::Scalar secondaryColor;
     cv::Scalar numberColor;
     float poolLength;
     float poolWidth;
     team();
     team(cv::Scalar primaryColor=cv::Scalar(0,0,0), cv::Scalar secondaryColor=cv::Scalar(255,255,255), cv::Scalar numberColor=cv::Scalar(255,255,255), float poolLength=25, float poolWidth=20);       // constructor
 };*/

team::team() {
    primaryColor = Scalar(0,0,0);
    secondaryColor=Scalar(255,255,255);
    numberColor=Scalar(255,255,255);
    poolLength=25;
    poolWidth=20;
}
team::team(Scalar primaryColor, Scalar secondaryColor, Scalar numberColor, float poolLength, float poolWidth) : primaryColor(primaryColor), secondaryColor(secondaryColor), numberColor(numberColor), poolLength(poolLength), poolWidth(poolWidth) {}

// GLOBAL VARIABLES this function will determine the location, home, and away teams
team location;
team home;
team away;
team ball;

void teamdata(string path)
{
    // use regular expression to parse data from the file path name
    string locationString;
    string awayString;
    string homeString;
    string dateString;
    regex location_expr("[a-z]+(?=.[a-z]+-)");
    regex away_expr ("[a-z]+(?=-)");
    regex home_expr ("[a-z]+(?=.[0-9])");
    regex date_expr ("[0-9]+");
    smatch m;
    regex_search(path, m, location_expr);
    for (auto x : m) {locationString.append(x);}
    regex_search(path, m, away_expr);
    for (auto x:m) {awayString.append(x);}
    regex_search(path, m, home_expr);
    for (auto x:m) {homeString.append(x);}
    regex_search(path, m, date_expr);
    for (auto x:m) {dateString.append(x);}
    
    // standard pool measurements [meters]
    float poolLengthMax = 25;
    float poolLengthMin = 22.9;
    float poolWidthMax = 20;
    float eightLanes = 17.04;
    float poolWidthMin = 13;
    //float floatingGoalLine2Ropes = 0.30;
    //float floatingGoalLine2Wall = 1.66;
    //float goalWidth = 3;
    //float goalHeight = 0.90;
    //float goalHeightShallow = 2.40;
    float ballRadius = .695/2/M_PI;
    
    // team data (primary color, secondary color, number color, pool length, pool width)
    team b(Scalar(0,171,231), Scalar(0,0,0), Scalar(0,0,0), ballRadius, 0); ball = b;
    team sluh(Scalar(162,72,0), Scalar(255,255,255), Scalar(255,255,255), poolLengthMin, poolWidthMin);
    team lindbergh(Scalar(61,111,0), Scalar(1,210,254), Scalar(255,255,255), poolLengthMax, poolWidthMax);
    team kirkwood(Scalar(58,68,229), Scalar(255,255,255), Scalar(255,255,255), poolLengthMax, eightLanes);
    team parkwaywest(Scalar(203,147,70), Scalar(36,42,151), Scalar(255,255,255), poolLengthMin, poolWidthMin);
    
    // assign the location, home team, and away team to different objects
    if (locationString == "sluh") {location = sluh;}
    else if (locationString == "lindbergh") {location = lindbergh;}
    else if (locationString == "kirkwood") {location = kirkwood;}
    else if (locationString == "parkwaywest") {location = parkwaywest;}
    if (awayString == "sluh") {away = sluh;}
    else if (awayString == "lindbergh") {away = lindbergh;}
    else if (awayString == "kirkwood") {away = kirkwood;}
    else if (awayString == "parkwaywest") {away = parkwaywest;}
    if (homeString == "sluh") {home = sluh;}
    else if (homeString == "lindbergh") {home = lindbergh;}
    else if (homeString == "kirkwood") {home = kirkwood;}
    else if (homeString == "parkwaywest") {home = parkwaywest;}
}
