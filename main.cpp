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

//### SUCCESSES ###
// inverse warping (p. 165) because findHomography does it for you

//### FAILURES ###
// belief propagation (p. 184) -- Markov Random Fields https://research.project-10.de/dgm/

//### IMPROVEMENTS ###
// make all variables non-global
// create easier way (perhaps a define statement) to have a debug mode and presentation mode

//### IDEAS ###
// Once we have the pool outline, iterate through the pixels in the pool to analyze with HSV, BGR, or some other method to locate players. Could try searching for anything which was not white or blue in the pool area to find players or the ball
// *feature tracking to track players and ball and perhaps camera movement (p. 235)
// 

//### TASKS ###
// add good comments to all code in the main function



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
VideoCapture cap("Resources/LindberghPool/video.mp4");
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

// DATA COLLECTION global variables
Point2f* ptrPlayer;
string inputState = "initialize";
bool inputPressed = false;
vector<Point2f> awayPlayers;
vector<Point2f> homePlayers;
Point2f ball;
bool WARNING_tooManyPlayers = false;
bool WARNING_noPlayersToMove = false;
bool WARNING_noPlayersToRemove = false;
int countWARNING_tooManyPlayers = 0;
int countWARNING_noPlayersToMove = 0;
int countWARNING_noPlayersToRemove = 0;
Mat cam;
vector<Point2f> pointsSide;
vector<Point2f> pointsCorner;
vector<string> possibleInputStates;
vector<Point2f> opticalFlowTrackPoints;

// DATA COLLECTION function declarations
double distance(Point2f a, Point2f b);
vector<Point2f> poolSides2Corners(vector<Point2f> sides);
void drawUserInput();
void UserInputCallBackFunc( int event, int x, int y, int flags, void* userdata);

// function declarations
vector<Point2f> getPlayerContours(Mat cam, Mat camHSVPrep);
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
vector<Point2f> findCamPoolCorners(Mat img, vector<Point2f> poolBoundary);



/*int main()      // DATA COLLECTION
{
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    bool finalRun = false;
    
    // initialize all player, ball, and pool boundary positions to be unknown (i.e. (-10,-10))
    for (int i=0; i<7; i++) {
        awayPlayers.push_back(Point2f(-10,-10));
        homePlayers.push_back(Point2f(-10,-10));
    }
    ball = Point2f(-10,-10);
    for (int i=0; i<8; i++) {pointsSide.push_back(Point2f(-10,-10));}
    for (int i=0; i<4; i++) {pointsCorner.push_back(Point2f(-10,-10));}
    
    // prepare video for frame-by-frame input with the callback function
    string path = "Resources/LindberghPool/video.mp4";
    VideoCapture cap(path);
    namedWindow("Camera");
    setMouseCallback("Camera", UserInputCallBackFunc);
    possibleInputStates = {"away +", "home +", "away move", "home move", "ball move", "sidelines move", "away -", "home -", "ball +/-", "optical flow"};
    
    // optical flow initialization
    // read in the first frame
    cap.read(cam);
    // take first frame and find corners in it
    Mat old_frame = cam.clone();
    Mat old_gray;
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    Mat cam_gray;
    vector<Point2f> new_opticalFlowTrackPoints;

    while (true) {
        // read in new frame
        cap.read(cam);
        if (cam.empty()) {break;}
        
        // calculate optical flow to detect camera movement for pool sides
        // general optical flow data
        cvtColor(cam, cam_gray, COLOR_BGR2GRAY);
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        // only perform the optical flow for camera movement if there are points to track
        if (opticalFlowTrackPoints.size()) {
            calcOpticalFlowPyrLK(old_gray, cam_gray, opticalFlowTrackPoints, new_opticalFlowTrackPoints, status, err, Size(15,15), 2, criteria);
            vector<Point2f> good_new, good_old;
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
            Mat globalMotionMatrix = cv::videostab::estimateGlobalMotionLeastSquares(good_old, good_new);
            // estimate the position of the pool sides based on camera movement
            for (int i=0; i<pointsSide.size(); i++) {
                float x = pointsSide[i].x*globalMotionMatrix.at<float>(0,0) + pointsSide[i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
                float y = pointsSide[i].x*globalMotionMatrix.at<float>(1,0) + pointsSide[i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
                float z = pointsSide[i].x*globalMotionMatrix.at<float>(2,0) + pointsSide[i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
                pointsSide[i] = Point2f(x/z,y/z);
            }
            pointsCorner = poolSides2Corners(pointsSide);
        }
        
        // calculate optical flow to detect camera movement for the players and the ball
        vector<Point2f> old_playersBall, new_playersBall;
        old_playersBall = awayPlayers;
        old_playersBall.insert(old_playersBall.end(), homePlayers.begin(), homePlayers.end());
        old_playersBall.push_back(ball);
        calcOpticalFlowPyrLK(old_gray, cam_gray, old_playersBall, new_playersBall, status, err, Size(15,15), 2, criteria);
        vector<Point2f> good_new, good_old;
        for(uint i = 0; i < old_playersBall.size(); i++)
        {
            if (i < awayPlayers.size()) {
                if (awayPlayers[i] == Point2f(-10,-10)) {continue;}
                else if (status[i] == 1) {awayPlayers[i] = new_playersBall[i];}
            } else if (i < (awayPlayers.size()+homePlayers.size())) {
                if (homePlayers[i-awayPlayers.size()] == Point2f(-10,-10)) {continue;}
                else if (status[i] == 1) {homePlayers[i-awayPlayers.size()] = new_playersBall[i];}
            } else {
                if (ball == Point2f(-10,-10)) {continue;}
                else if (status[i] == 1) {ball = new_playersBall[i];}
            }
        }
        old_gray = cam_gray.clone();
        
        // wait for user input to determine the state or to move on to the next frame
        while (true) {
            drawUserInput();
            int key = waitKey(10);
            if (inputState == "initialize") {continue;}
            if (key == 'c') {break;}
            if (key == '1') {inputState = possibleInputStates[0];}
            if (key == '2') {inputState = possibleInputStates[1];}
            if (key == '3') {inputState = possibleInputStates[2];}
            if (key == '4') {inputState = possibleInputStates[3];}
            if (key == '5') {inputState = possibleInputStates[4];}
            if (key == '6') {inputState = possibleInputStates[5];}
            if (key == '7') {inputState = possibleInputStates[6];}
            if (key == '8') {inputState = possibleInputStates[7];}
            if (key == '9') {inputState = possibleInputStates[8];}
            if (key == '0') {inputState = possibleInputStates[9];}
            if (key == 27) {finalRun = true;}
        }
        
        // store the current positions of the players into the vectors which store their positions for all time
        awayPlayers_vector.push_back(awayPlayers);
        homePlayers_vector.push_back(homePlayers);
        ball_vector.push_back(ball);
        sides_vector.push_back(pointsSide);
        corners_vector.push_back(pointsCorner);
        
        if (finalRun) {break;}
    }
    
    // save data to files
    ofstream awayPlayers_file("Files/awayPlayers");
    ofstream homePlayers_file("Files/homePlayers");
    ofstream ball_file("Files/ball");
    ofstream sides_file("Files/sides");
    ofstream corners_file("Files/corners");
    ostream_iterator<vector<Point2f>> awayPlayers_iterator(awayPlayers_file, "\n" );
    ostream_iterator<vector<Point2f>> homePlayers_iterator(homePlayers_file, "\n" );
    ostream_iterator<Point2f> ball_iterator(ball_file, "\n" );
    ostream_iterator<vector<Point2f>> sides_iterator(sides_file, "\n" );
    ostream_iterator<vector<Point2f>> corners_iterator(corners_file, "\n" );
    copy(awayPlayers_vector.begin( ), awayPlayers_vector.end( ), awayPlayers_iterator);
    copy(homePlayers_vector.begin( ), homePlayers_vector.end( ), homePlayers_iterator);
    copy(ball_vector.begin( ), ball_vector.end( ), ball_iterator);
    copy(sides_vector.begin( ), sides_vector.end( ), sides_iterator);
    copy(corners_vector.begin( ), corners_vector.end( ), corners_iterator);
    
    // ??? make old files re-editable
}*/


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
    imshow("Camera", cam);
}

void drawAnimated_PRESENTATION(int iterator, vector<vector<Point2f>> awayPlayers_vector, vector<vector<Point2f>> homePlayers_vector, vector<Point2f> ball_vector, vector<vector<Point2f>> corners_vector) {
    
    Mat pool;
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
    
    // draw the pool
    pool.create(windowHeight, windowLength, CV_8UC3);
    pool.setTo(poolBlue);                      // create a blank pool canvas
    line(pool, Point(0,windowHeight/2-windowHeight*goalWidth/poolWidth/2), Point(0,windowHeight/2+windowHeight*goalWidth/poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength,windowHeight/2-windowHeight*goalWidth/poolWidth/2), Point(windowLength,windowHeight/2+windowHeight*goalWidth/poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength*2/poolLength,0), Point(windowLength*2/poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength-windowLength*2/poolLength,0), Point(windowLength-windowLength*2/poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength*6/poolLength,0), Point(windowLength*6/poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength-windowLength*6/poolLength,0), Point(windowLength-windowLength*6/poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength*half/poolLength,0), Point(windowLength*half/poolLength,windowHeight), poolWhite,3);      // half pool

    // circle on each player and ball
    for (int i=0; i<awayPlayers_animated.size(); i++) {
        if (awayPlayers_animated[i] != Point2f(-10,-10)) {circle(pool, awayPlayers_animated[i], 5, awayColor, FILLED);}
        if (homePlayers_animated[i] != Point2f(-10,-10)) {circle(pool, homePlayers_animated[i], 5, homeColor, FILLED);}
    }
    if (ball_animated != Point2f(-10,-10)) {circle(pool, ball_animated, 5, Scalar(0,255,255), FILLED);}
    
    // imshow animated pool
    imshow("animated pool", pool);
}


int main()      // PRESENTATION
{
    Mat cam;
    string path = "Resources/LindberghPool/video.mp4";                          // VIDEO ONLY
    VideoCapture cap(path);                                                     // VIDEO ONLY
    
    // read in data
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    ifstream awayPlayers_file("Files/awayPlayers");
    ifstream homePlayers_file("Files/homePlayers");
    ifstream ball_file("Files/ball");
    ifstream sides_file("Files/sides");
    ifstream corners_file("Files/corners");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16;
    char c;
    while (awayPlayers_file >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_file >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_file >> c >> var1 >> c >> var2 >> c) {ball_vector.push_back(Point2f(var1,var2));}
    while (sides_file >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> var16 >> c) {sides_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (corners_file >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    
    // show the video
    unsigned int iterator = 0;
    while (true) {
        cap.read(cam);
        if (cam.empty()) {break;}
        
        drawCamera_PRESENTATION(cam, iterator, awayPlayers_vector, homePlayers_vector, ball_vector, corners_vector);
        drawAnimated_PRESENTATION(iterator, awayPlayers_vector, homePlayers_vector, ball_vector, corners_vector);
        iterator++;
        
        int key = waitKey(1);
        if (key == 27) {return 0;}
    }
}

/*int main()
{
    // declare Mat variables (images)
    Mat cam, animPool;    // camera image
    animPool = Mat(500, 500, CV_8UC3, Scalar(255,255,255));
    Mat camHisteq, camHSVcolor, camHSV, camHSVMed, camHSVMedDil, camHSVPrep;        // preprocessing for HSV-filtered pool for (getContours)
    Mat camGauss, camGaussGray, camCanny;                                           // preprocessing for Canny lines pool
    Mat camPlayerHSV, camPlayerHSVMed, camPlayerHSVMedDil, camPlayerHSVPrep;
    Mat camCannyHough;
    Mat waitbar = Mat(200, 500, CV_8UC3);                                       // SAVE-VIDEO ONLY
    
    // locate the position of the mouse on a window
    //namedWindow("camera");
    //setMouseCallback("camera", LocationCallBackFunc);
    
    // setup the capture method, either picture or video
    //string path = "Resources/LindberghPool/pos2.png";                           // PICTURE ONLY
    string path = "Resources/LindberghPool/video.mp4";                          // VIDEO ONLY
    VideoCapture cap(path);                                                     // VIDEO ONLY
    cap.read(cam);                                                              // VIDEO ONLY
    
    //int frame_width = static_cast<int>(cap.get(CAP_PROP_FRAME_WIDTH));          // SAVE-VIDEO ONLY*
    //int frame_height = static_cast<int>(cap.get(CAP_PROP_FRAME_HEIGHT));
    //int frame_count = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    //unsigned long frameCounter = 0;
    //Size frame_size(frame_width, frame_height);
    //int frames_per_second = 10;
    //VideoWriter camVideoWriter("Resources/LindberghPool/outputCAM.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),frames_per_second, frame_size, true);
    //VideoWriter animVideoWriter("Resources/LindberghPool/outputANIM.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'),frames_per_second, Size(500,400), true);
    
    // create two trackbars: colorspace and preprocessing
    // colorspace allows adjustment of the HSV values to find the blue water of the pool, filtering so as to get as near to the pool boundary as possible while not including anything outside the pool
    // preprocessing allows the adjustment of the kernel size for median blur, dilation, and erosion of the HSV-filtered image (camHSV) to produce (camHSVPrep), which is used by (getContours) to approximate the inner boundary of the pool
    // preprocessing allows the adjustment of the Canny lower threshold value and the Hough lower threshold value to find the Canny lines of the pool boundary
    trackbar track1("colorspace",{"H","S","V"},{180,255,255});
    track1.set_initialvalues({78,0,36}, {120,255,255});                             // POOL-SPECIFIC
        // Lindbergh High School: {78,0,36}, {120,255,255}
    trackbar track2("preprocessing",{"Blur","Dilate","Erode","Canny","Hough"},{20,20,20,100,400});
    track2.set_initialvalues({4,2,20,30,170}, track2.maxValue);                     // POOL-SPECIFIC
        // Lindbergh High School: {4,2,20,30,170}, track2.maxValue
    trackbar track3("players",{"H","S","V","Blur","Dilate","Erode"},{180,255,255,20,20,20});
    track3.set_initialvalues({0,0,0,0,0,0}, {36,255,255,20,20,20});                     // POOL-SPECIFIC
    
    // keep track of where the four sides (northern, southern,...) of the pool were on the last iteration, with each side of the pool formed by a line connecting two points (x1,y1) (x2,y2)
    // for the first iteration, initialize the sides of the camera window as the four sides of the pool
    Vec4i northPrev =  {0, 0, cam.cols, 0};
    Vec4i eastPrev = {cam.cols, 0, cam.cols, cam.rows};
    Vec4i southPrev = {0, cam.rows, cam.cols, cam.rows};
    Vec4i westPrev = {0, 0, 0, cam.rows};
    
    // keep count of the number of times the sides of the pool on the last iteration are the exact same as they are on the current iteration
    // start at 0, and if they are the same 5 times in a row, then set the side of the pool on the last iteration to be the camera boundary
    int northCount = 0;
    int eastCount = 0;
    int southCount = 0;
    int westCount = 0;
    int sameSideLimit = 5;
    
    // NEW SHIT
    //poolIdentifierImage = cam.clone();
    frame = cam.clone();
    namedWindow("pool state picker");
    namedWindow("Camera");
    setMouseCallback("pool state picker", UserPoolStateCallBackFunc);
    setMouseCallback("Camera", UserPoolIdentifierCallBackFunc);
    Mat poolStatePicker = Mat(500, 100, CV_8SC3);
    poolStatePicker.setTo(Scalar(255,255,255));
    vector<string> deformation = {"2M","5M","Half","Corner","Side"};
    while (true) {  // this lets you pick the sides of the pool
        for (int i=0; i<deformation.size(); i++)         // draw buttons
        {
            putText(poolStatePicker, deformation[i], Point(100*0.1,500*(i/5.0+0.1)), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
            rectangle(poolStatePicker, Point(0,500*i/5.0), Point(100,500*(i/5.0+1.0/deformation.size())), Scalar(0,0,0), 2);
        }
        
        if (buttonPressed < deformation.size()) {        // if a button is pressed, make it blue instead of red
            putText(poolStatePicker, deformation[buttonPressed], Point(100*0.1,500*(buttonPressed/5.0+0.1)), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,0,0), 2);
        }
        
        
        imshow("pool state picker", poolStatePicker);
        imshow("Camera", frame);
        int k = waitKey(10);
        if (k == 27) {break;}
    }
    
    findPoolCornersFromUserInput(cam);
    
    // Create some random colors
    vector<Scalar> colors;
    RNG rng;
    for(int i = 0; i < 100; i++)
    {
        int r = rng.uniform(0, 256);
        int g = rng.uniform(0, 256);
        int b = rng.uniform(0, 256);
        colors.push_back(Scalar(r,g,b));
    }
    Mat old_frame, old_gray;
    vector<Point2f> p1;
    // Take first frame and find corners in it
    old_frame = cam.clone();
    cvtColor(old_frame, old_gray, COLOR_BGR2GRAY);
    //goodFeaturesToTrack(old_gray, p0, 100, 0.3, 7, Mat(), 7, false, 0.04);
    namedWindow("pick the points to track");
    setMouseCallback("pick the points to track", OpticalFlowCallBackFunc, NULL);
    while (waitKey(30) != 27) {     // this lets you pick the points to track
        imshow("pick the points to track", old_frame);
        for (int i=0; i<p0.size(); i++) {
            circle(old_frame, p0[i], 5, colors[i], -1);
        }
        for (int i=0; i<temporaryPoolCorners.size(); i++) {
            circle(old_frame, temporaryPoolCorners[i], 15, Scalar(0,0,255), -1);
        }
    }
    
    
    // Create a mask image for drawing purposes
    Mat mask = Mat::zeros(old_frame.size(), old_frame.type());
    Mat frame_gray;
    //Mat frame, frame_gray;
    
    
    doneInitializing = true;
    while (true) {      // this tracks the points
        cap.read(frame);                                                          // VIDEO ONLY
        if (frame.empty()) {break;}
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        
        
        if (p0.size()<5) {
            while (waitKey(30) != 27) {     // this lets you pick the points to track
                imshow("pick the points to track", frame);
                for (int i=0; i<p0.size(); i++) {
                    circle(frame, p0[i], 5, colors[i], -1);
                }
                for (int i=0; i<temporaryPoolCorners.size(); i++) {
                    circle(old_frame, temporaryPoolCorners[i], 15, Scalar(0,0,255), -1);
                }
            }
        }
        
        // calculate optical flow
        vector<uchar> status;
        vector<float> err;
        TermCriteria criteria = TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 10, 0.03);
        calcOpticalFlowPyrLK(old_gray, frame_gray, p0, p1, status, err, Size(15,15), 2, criteria);
        vector<Point2f> good_new, good_old;
        int goodPointCounter = 0;
        for(uint i = 0; i < p0.size(); i++)
        {
            // Select good points
            if(status[i] == 1) {
                good_old.push_back(p0[i]);
                good_new.push_back(p1[i]);
                goodPointCounter++;
                // draw the tracks
                //line(mask,p1[i], p0[i], colors[i], 2);
                //circle(frame, p1[i], 5, colors[i], -1);
            }
        }
        
        // Now update the previous frame and previous points
        old_gray = frame_gray.clone();
        p0 = good_new;
        Mat globalMotionMatrix = cv::videostab::estimateGlobalMotionLeastSquares(good_old, good_new);
        
        p = Point(0,0);
        for (int i=0; i<pointsSide.size(); i++) {
            float x = pointsSide[i].x*globalMotionMatrix.at<float>(0,0) + pointsSide[i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
            float y = pointsSide[i].x*globalMotionMatrix.at<float>(1,0) + pointsSide[i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
            float z = pointsSide[i].x*globalMotionMatrix.at<float>(2,0) + pointsSide[i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
            float xVal, yVal;
            xVal = x/z; yVal = y/z;
            pointsSide[i] = Point2f(xVal,yVal);
            
            // centroid of the pool
            circle(frame, pointsSide[i], 5, Scalar(0,0,0), -1);
            p.x += xVal; p.y += yVal;
            //cout<<"x: "<<x<<"   y: "<<y<<"   z: "<<z<<"   poolCorner: "<<temporaryPoolCorners[i]<<endl;
        }
        p.x = p.x / pointsSide.size();
        p.y = p.y / pointsSide.size();
        //cout<<globalMotionMatrix<<endl;
        
        //for (int i=0; i<temporaryPoolCorners.size(); i++) {
         //   float x = temporaryPoolCorners[i].x*globalMotionMatrix.at<float>(0,0) + temporaryPoolCorners[i].y*globalMotionMatrix.at<float>(0,1) + globalMotionMatrix.at<float>(0,2);
         //   float y = temporaryPoolCorners[i].x*globalMotionMatrix.at<float>(1,0) + temporaryPoolCorners[i].y*globalMotionMatrix.at<float>(1,1) + globalMotionMatrix.at<float>(1,2);
         //   float z = temporaryPoolCorners[i].x*globalMotionMatrix.at<float>(2,0) + temporaryPoolCorners[i].y*globalMotionMatrix.at<float>(2,1) + globalMotionMatrix.at<float>(2,2);
         //   float xVal, yVal;
         //   xVal = x/z; yVal = y/z;
         //   temporaryPoolCorners[i] = Point2f(xVal,yVal);
        //    cout<<"x: "<<x<<"   y: "<<y<<"   z: "<<z<<"   poolCorner: "<<temporaryPoolCorners[i]<<endl;
        //}
        //for (int i=0; i<temporaryPoolCorners.size(); i++) {
        //    circle(frame, temporaryPoolCorners[i], 15, Scalar(0,0,255), -1);
            //if (temporaryPoolCorners[i].x<0) {
            //    putText(frame, to_string(i), Point(20,temporaryPoolCorners[i].y), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,0),3);
            //} else if (temporaryPoolCorners[i].x>frame.cols) {
            //    putText(frame, to_string(i), Point(frame.cols-20,temporaryPoolCorners[i].y), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,0),3);
            //} else {
            //    putText(frame, to_string(i), temporaryPoolCorners[i], FONT_HERSHEY_PLAIN, 5, Scalar(0,0,0),3);
            //}
        //    line(frame, temporaryPoolCorners[i], temporaryPoolCorners[(i+1)%4], Scalar(0,0,0), 3);
        //}
        Mat img;
        
        
        //if (temporaryPoolCorners.size()==4) {
        //    Point2f holder1ForCameraPoolCorners = temporaryPoolCorners[0];
        //    Point2f holder2ForCameraPoolCorners = temporaryPoolCorners[1];
        //    temporaryPoolCorners[0] = temporaryPoolCorners[2];
        //    temporaryPoolCorners[1] = temporaryPoolCorners[3];
        //    temporaryPoolCorners[2] = holder1ForCameraPoolCorners;
        //    temporaryPoolCorners[3] = holder2ForCameraPoolCorners;
        //}
        Vec4i vec4iLine0 = drawLongLine(frame, {static_cast<int>(pointsSide[0].x),static_cast<int>(pointsSide[0].y),static_cast<int>(pointsSide[1].x),static_cast<int>(pointsSide[1].y)});
        Vec4i vec4iLine1 = drawLongLine(frame, {static_cast<int>(pointsSide[2].x),static_cast<int>(pointsSide[2].y),static_cast<int>(pointsSide[3].x),static_cast<int>(pointsSide[3].y)});
        Vec4i vec4iLine2 = drawLongLine(frame, {static_cast<int>(pointsSide[4].x),static_cast<int>(pointsSide[4].y),static_cast<int>(pointsSide[5].x),static_cast<int>(pointsSide[5].y)});
        Vec4i vec4iLine3 = drawLongLine(frame, {static_cast<int>(pointsSide[6].x),static_cast<int>(pointsSide[6].y),static_cast<int>(pointsSide[7].x),static_cast<int>(pointsSide[7].y)});
        
        vector<Point2f> poolBoundary = findCamPoolBoundary(frame);
        findPoolCornersFromUserInput(frame);
        //vector<Point2f> cameraPoolCorners = findCamPoolCorners(frame, poolBoundary);
        if (cornerPoints.size() == 4) {
            animPool = drawAnimatedPool(cornerPoints, poolBoundary);
            cornerPt1_vector.push_back(cornerPoints[0]);
            cornerPt2_vector.push_back(cornerPoints[1]);
            cornerPt3_vector.push_back(cornerPoints[2]);
            cornerPt4_vector.push_back(cornerPoints[3]);
        }
        poolLines.clear();
        
        // Show results
        add(frame, mask, img);
        //imshow("camera",cam);                                                     // ~SAVE-VIDEO ONLY
        imshow("animated pool", animPool);                                        // ~SAVE-VIDEO ONLY
        imshow("Camera", img);
        int keyboard = waitKey(30);
        if (keyboard == 27) {break;}
        else if (keyboard == 'p') {waitKey(0);}
    }
    
    ofstream cornerPt1_file("Files/corner1");
    ofstream cornerPt2_file("Files/corner2");
    ofstream cornerPt3_file("Files/corner3");
    ofstream cornerPt4_file("Files/corner4");
    ostream_iterator<Point2f> cornerPt1_iterator(cornerPt1_file, "\n" );
    ostream_iterator<Point2f> cornerPt2_iterator(cornerPt2_file, "\n" );
    ostream_iterator<Point2f> cornerPt3_iterator(cornerPt3_file, "\n" );
    ostream_iterator<Point2f> cornerPt4_iterator(cornerPt4_file, "\n" );
    // Passing all the variables inside the vector from the beginning of the vector to the end.
    copy(cornerPt1_vector.begin( ), cornerPt1_vector.end( ), cornerPt1_iterator);
    copy(cornerPt2_vector.begin( ), cornerPt2_vector.end( ), cornerPt2_iterator);
    copy(cornerPt3_vector.begin( ), cornerPt3_vector.end( ), cornerPt3_iterator);
    copy(cornerPt4_vector.begin( ), cornerPt4_vector.end( ), cornerPt4_iterator);
}*/
    
    
    /*while (true) {
        cap.read(cam);                                                          // VIDEO ONLY
        if (cam.empty()) {break;}
        
        
        
        
        
        
        imshow("camera",cam);                                                     // ~SAVE-VIDEO ONLY
        imshow("animated pool", animPool);                                        // ~SAVE-VIDEO ONLY
        
        // End the program when the "ESC" key is pressed
        int key=waitKey(10);
        if (key==27) {
            //camVideoWriter.release(); //flush and close the video file    // SAVE-VIDEO ONLY
            //animVideoWriter.release(); //flush and close the video file    // SAVE-VIDEO ONLY
            return 0;
        } else if (key==112) {waitKey(0);}
    }*/
    
    
    /*// ### KALMAN: SETUP ###
    
    // array sizes and number of iterations
    const int n_x = 8;      // {x1, y1, x2, y2, x3, y3, x4, y4}
    const int n_u = 1;
    const int n_z = 1;
    const int numberOfIterations = 30;
    
    // input values of constant matrices
    double H_array[n_z][n_x] = {
        {1, 0}};
    double F_array[n_x][n_x] = {
        {1, 0.25},
        {0, 1}};
    double G_array[n_x][n_u] = {
        {0.0313},
        {0.25}};
    double Q_array[n_x][n_x] = {
        {pow(0.25,4)/4*pow(0.1,2), pow(0.25,3)/2*pow(0.1,2)},
        {pow(0.25,3)/2*pow(0.1,2), pow(0.25,2)*pow(0.1,2)}};
    double R_array[n_z][n_z] = {
        {400}};
    
    // create matrices
    cv::Mat x_hat = cv::Mat(n_x, 1, CV_64F);
    cv::Mat u = cv::Mat(n_u, 1, CV_64F);
    cv::Mat P = cv::Mat(n_x, n_x, CV_64F);
    cv::Mat F = cv::Mat(n_x, n_x, CV_64F, F_array);
    cv::Mat G = cv::Mat(n_x, n_u, CV_64F, G_array);
    cv::Mat Q = cv::Mat(n_x, n_x, CV_64F, Q_array);
    cv::Mat H = cv::Mat(n_z, n_x, CV_64F, H_array);
    cv::Mat z = cv::Mat(n_z, 1, CV_64F);
    cv::Mat R = cv::Mat(n_z, n_z, CV_64F, R_array);
    cv::Mat K = cv::Mat(n_x, n_z, CV_64F);
    cv::Mat I = cv::Mat::eye(n_x, n_x, CV_64F);
    
    // array of measured values
    double measurements[30] = {-32.4, -11.1, 18, 22.9, 19.5, 28.5, 46.5, 68.9, 48.2, 56.1, 90.5, 104.9, 140.9, 148, 187.6, 209.2, 244.6, 276.4, 323.5, 357.3, 357.4, 398.3, 446.7, 465.1, 529.4, 570.4, 636.8, 693.3, 707.3, 748.5};
    double controlVariables[30] = {39.72, 40.02, 39.97, 39.81, 39.75, 39.6, 39.77, 39.83, 39.73, 39.87, 39.81, 39.92, 39.78, 39.98, 39.76, 39.86, 39.61, 39.86, 39.74, 39.87, 39.63, 39.67, 39.96, 39.8, 39.89, 39.85, 39.9, 39.81, 39.81, 39.68};
    
    
    
    // ### KALMAN: INITIALIZE ###
    
    // Input: System State Initial Guess
    double x_hat_array[n_x][1] = {
        {0},
        {0}};
    x_hat = cv::Mat(n_x, 1, CV_64F, x_hat_array);
    double u_array[n_u][1] = {
        {9.8}};
    u = cv::Mat(n_u, 1, CV_64F, u_array);
    
    // Input: Estimate Uncertainty Initial Guess
    double P_array[n_x][n_x] = {
        {500, 0},
        {0, 500}};
    P = cv::Mat(n_x, n_x, CV_64F, P_array);*/
    
    // repetitively loops through the camera frames until there are none left or the "ESC" key is pressed
    /*while (true) {
        
        // read the next camera frame in as (cam); if empty then break out of the while loop
        //cam = imread(path);                                                     // PICTURE ONLY
        cap.read(cam);                                                          // VIDEO ONLY
        if (cam.empty()) {break;}
        
        // HSV filter the camera frame (cam) to focus in on the inner boundary of the pool, producing a binary image (camHSV)
        camHisteq = histogramEqualization(cam);             // optimizes the brightness and intensity values of (cam)
        cvtColor(camHisteq, camHSVcolor, COLOR_BGR2HSV);    // need to convert BGR to HSV colorspace to filter the pool
        inRange(camHSVcolor,track1.lowerBound,track1.upperBound,camHSV);
        
        // preprocessing the HSV-filtered image to prepare for (getContours) which closely estimates the inner pool boundary
        medianBlur(camHSV, camHSVMed, track2.minValue[0]*2+1);
        dilate(camHSVMed, camHSVMedDil, getStructuringElement(MORPH_RECT, Size(track2.minValue[1]*2+1,track2.minValue[1]*2+1)));
        erode(camHSVMedDil, camHSVPrep, getStructuringElement(MORPH_RECT, Size(track2.minValue[2]*2+1,track2.minValue[2]*2+1)));
        
        // preprocessing for the players
        inRange(camHSVcolor,track3.lowerBound,track3.upperBound,camPlayerHSV);
        medianBlur(camPlayerHSV, camPlayerHSVMed, track3.minValue[3]*2+1);
        dilate(camPlayerHSVMed, camPlayerHSVMedDil, getStructuringElement(MORPH_RECT, Size(track3.minValue[4]*2+1,track3.minValue[4]*2+1)));
        erode(camPlayerHSVMedDil, camPlayerHSVPrep, getStructuringElement(MORPH_RECT, Size(track3.minValue[5]*2+1,track3.minValue[5]*2+1)));
        
        // preprocessing the camera image for Canny edge detection
        GaussianBlur(cam, camGauss, Size(track2.minValue[0]*2+1,track2.minValue[0]*2+1), 0);
        cvtColor(camGauss, camGaussGray, COLOR_BGR2GRAY);
        
        // Canny edge detection and overlaying the HSV-filtered pool as a black shape onto the Canny lines image
        // this resulting image in BGR will be used for the Hough probabilitic line transform
        Canny(camGaussGray, camCanny, track2.minValue[3], track2.minValue[3]*2.5);
        getContours(cam, camHSVPrep, camCanny);
        cvtColor(camCanny, camCannyHough, COLOR_GRAY2BGR);
        
        // use the Hough probabilistic line transform to obtain the probable lines from (camCanny) and store them in (linesP)
        vector<Vec4i> linesP;                                                       // will hold the results of the detection
        HoughLinesP(camCanny, linesP, 1, CV_PI/180, track2.minValue[4], 50, 10 );   // runs the actual detection
        
        // set the four sides of the pool to be equal to what they were previously, with a maximum distance from the pool centroid
        //double maxDistance = sqrt(cam.cols*cam.cols+cam.rows*cam.rows);
        //Vec4i north = northPrev; double northDist = maxDistance;
        //Vec4i south = southPrev; double southDist = maxDistance;
        //Vec4i west = westPrev; double westDist = maxDistance;
        //Vec4i east = eastPrev; double eastDist = maxDistance;
        
        // iterate through each line of (linesP), to see if it matches the criteria for a side of the pool (north, east, south, or west based on the position of the two endpoints and the angle of the line)
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            Vec4i l = linesP[i];
            // line(camCannyHough, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 2);     // DEBUG MODE
            double angle = atan(static_cast<double>(l[3]-l[1])/static_cast<double>(l[2]-l[0]))*180/M_PI;
            if (l[1]<p.y && l[3]<p.y && angle<10 && angle>-10) {         // north
                double dist = distancePoint2Line(p, Point(l[0], l[1]), Point(l[2], l[3]));
                if (dist < northDist) {
                    northDist = dist;
                    north = l;
                }
            } else if (l[1]>p.y && l[3]>p.y && angle<10 && angle>-10) {  // south
                double dist = distancePoint2Line(p, Point(l[0], l[1]), Point(l[2], l[3]));
                if (dist < southDist) {
                    southDist = dist;
                    south = l;
                }
            } else if (l[0]<p.x && l[2]<p.x && (l[1]>north[1] || l[3]>north[1]) && (l[1]<south[1] || l[3]<south[1]) && (angle<-10 || angle>10)) {  // west
                double dist = distancePoint2Line(p, Point(l[0], l[1]), Point(l[2], l[3]));
                //cout<<"west: "<<l[0]<<" < "<<p.x<<"   and   "<<l[2]<<" < "<<p.x<<"   and   angle: "<<angle<<endl;
                if (dist < westDist) {
                    westDist = dist;
                    west = l;
                }
            } else if (l[0]>p.x && l[2]>p.x && (l[1]>north[1] || l[3]>north[1]) && (l[1]<south[1] || l[3]<south[1]) && (angle<-10 || angle>10)) {  // east
                double dist = distancePoint2Line(p, Point(l[0], l[1]), Point(l[2], l[3]));
                //cout<<"east: "<<l[0]<<" > "<<p.x<<"   and   "<<l[2]<<" > "<<p.x<<"   and   angle: "<<angle<<endl;
                if (dist < eastDist) {
                    eastDist = dist;
                    east = l;
                }
            }
        }
        
        // using the endpoints of the lines that represent the four sides of the pool, calculate the slope and y-intercept, storing them in the variable (poolLines)
        // draw the lines, which are not trimmed to size, on (camCannyHough)
        // store the old endpoints for the lines of the sides of the pool into their respective "previous" values
        //northPrev = drawLongLine(camCannyHough, north);
        //southPrev = drawLongLine(camCannyHough, south);
        //westPrev = drawLongLine(camCannyHough, west);
        //eastPrev = drawLongLine(camCannyHough, east);
        //circle(camCannyHough, p, 10, Scalar(0,0,255), -1); // show the image with a point mark at the centroid        // DEBUG MODE
        
        // if the sides of the pool have not changed for (sameSideLimit) iterations, then reset the respective side of the pool to be the camera boundary
        if (northPrev == north) {northCount++; if (northCount == sameSideLimit) {northCount=0; northPrev={0, 0, cam.cols, 0};}}
        else {northCount = 0;}
        if (southPrev == south) {southCount++; if (southCount == sameSideLimit) {southCount=0; southPrev={0, cam.rows, cam.cols, cam.rows};}}
        else {southCount = 0;}
        if (westPrev == west) {westCount++; if (westCount == sameSideLimit) {westCount=0; westPrev={0, 0, 0, cam.rows};}}
        else {westCount = 0;}
        if (eastPrev == east) {eastCount++; if (eastCount == sameSideLimit) {eastCount=0; eastPrev={cam.cols, 0, cam.cols, cam.rows};}}
        else {eastCount = 0;}
        
        // using the variable (poolLines) which has all of the probabilistic line slope-intercept values, find and plot the pool boundary lines which connect the pool corner points in (poolBoundary)
        vector<Point2f> poolBoundary = findCamPoolBoundary(cam);
        vector<Point2f> cameraPoolCorners = findCamPoolCorners(cam, poolBoundary);
        //vector<Point2f> camPlayerLocations = getPlayerContours(cam, camPlayerHSVPrep);
        if (cameraPoolCorners.size() == 4) {animPool = drawAnimatedPool(cameraPoolCorners, poolBoundary);}//, camPlayerLocations);}
        poolLines.clear();
        
        // Show results
        imshow("camera",cam);                                                     // ~SAVE-VIDEO ONLY
        imshow("animated pool", animPool);                                        // ~SAVE-VIDEO ONLY
        //camVideoWriter.write(cam);  //write the video frame to the file           // SAVE-VIDEO ONLY
        //animVideoWriter.write(animPool);                                            // SAVE-VIDEO ONLY
        //imshow(track1.windowName, track1.window);
        //imshow(track2.windowName, track2.window);
        //imshow(track3.windowName, track3.window);
        //imshow("camPlayerHSVPrep",camPlayerHSVPrep);
        //imshow("image mask",mask);
        //imshow("preprocessing",imgErode);
        //imshow("Detected Lines (in red) - Probabilistic Line Transform", camCannyHough);
        
        // waitbar for saving a video                                               // SAVE-VIDEO ONLY*
        frameCounter++;
        waitbar.setTo(cv::Scalar(175,175,175));
        rectangle(waitbar,Point(50,80),Point(400,120),Scalar(0,0,0),1); // black outline
        rectangle(waitbar,Point(51,81),Point(399,119),Scalar(255,255,255),FILLED); // white box fill
        double percentFinished = static_cast<double>(frameCounter)/frame_count;
        rectangle(waitbar, Point(51,81), Point(51+(399.0-51.0)*percentFinished,119), Scalar(0,0,255),FILLED); // red box fill
        putText(waitbar, to_string(static_cast<int>(100*percentFinished))+"%", Point(420,110), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),1.5);
        imshow("waitbar",waitbar);
        
        // End the program when the "ESC" key is pressed
        int key=waitKey(10);
        if (key==27) {
            //camVideoWriter.release(); //flush and close the video file    // SAVE-VIDEO ONLY
            //animVideoWriter.release(); //flush and close the video file    // SAVE-VIDEO ONLY
            return 0;
        } else if (key==112) {waitKey(0);}
    }

    //camVideoWriter.release(); //flush and close the video file        // SAVE-VIDEO ONLY
    //animVideoWriter.release(); //flush and close the video file        // SAVE-VIDEO ONLY
    return 0;
}*/






















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
    
    return corners;
}

void drawUserInput() {
    Scalar awayColor = Scalar(255,255,255);
    Scalar homeColor = Scalar(0,255,0);
    Mat camMask = cam.clone();
    Mat camMaskPositive = Mat::zeros(cam.size(), cam.type());
    Mat camMaskNegative = Mat::zeros(cam.size(), cam.type());
    
    // circle on each player and ball
    for (int i=0; i<awayPlayers.size(); i++) {
        circle(camMask, awayPlayers[i], 5, awayColor, FILLED);
        circle(camMask, homePlayers[i], 5, homeColor, FILLED);
    }
    circle(camMask, ball, 5, Scalar(0,255,255), FILLED);
    
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
    
    // black bar on bottom
    rectangle(camMask, Point(0,cam.rows-50), Point(cam.cols,cam.rows), Scalar(0,0,0), FILLED);
    
    // text on black bar
    if (inputState == "initialize") {
        int numberSidesRemaining = 0;
        for (int i=0; i<pointsSide.size(); i++) {
            if (pointsSide[i] == Point2f(-10,-10)) {numberSidesRemaining++;}
        }
        if (numberSidesRemaining == 0) {
            inputState = "";
            pointsCorner = poolSides2Corners(pointsSide);
        }
        putText(camMask, "Instructions: ", Point(cam.cols*4.0/10.0-70, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
        putText(camMask, "Choose "+to_string(numberSidesRemaining)+" remaining sides of pool", Point(cam.cols*4.0/10.0+70,cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);
    } else {
        for (int i=0; i<possibleInputStates.size(); i++) {
            if (inputState == possibleInputStates[i]) {putText(camMask, to_string((i+1)%possibleInputStates.size())+": "+possibleInputStates[i], Point(cam.cols*i/static_cast<double>(possibleInputStates.size())+20, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);
            } else {
            putText(camMask, to_string((i+1)%10)+": "+possibleInputStates[i], Point(cam.cols*i/static_cast<double>(possibleInputStates.size())+20, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);
            }
        }
        int awayPlayersCount = 0;
        int homePlayersCount = 0;
        for (int i=0; i<awayPlayers.size(); i++) {
            if (awayPlayers[i] != Point2f(-10,-10)) {awayPlayersCount++;}
            if (homePlayers[i] != Point2f(-10,-10)) {homePlayersCount++;}
        }
        if (inputState == "away +") {putText(camMask, "("+to_string(awayPlayersCount)+"/"+to_string(awayPlayers.size())+")", Point(cam.cols*1/static_cast<double>(possibleInputStates.size())-50, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
        else {putText(camMask, "("+to_string(awayPlayersCount)+"/"+to_string(awayPlayers.size())+")", Point(cam.cols*1/static_cast<double>(possibleInputStates.size())-50, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
        if (inputState == "home +") {putText(camMask, "("+to_string(homePlayersCount)+"/"+to_string(homePlayers.size())+")", Point(cam.cols*2/static_cast<double>(possibleInputStates.size())-50, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
        else {putText(camMask, "("+to_string(homePlayersCount)+"/"+to_string(homePlayers.size())+")", Point(cam.cols*2/static_cast<double>(possibleInputStates.size())-50, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
        if (inputState == "optical flow") {putText(camMask, "("+to_string(opticalFlowTrackPoints.size())+")", Point(cam.cols-50, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,255), 1);}
        else {putText(camMask, "("+to_string(opticalFlowTrackPoints.size())+")", Point(cam.cols-50, cam.rows-25), FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255), 1);}
    }
    
    // display any warnings
    if (WARNING_tooManyPlayers) {putText(camMask, "WARNING: too many players", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_tooManyPlayers++; if(countWARNING_tooManyPlayers>10){WARNING_tooManyPlayers = false; countWARNING_tooManyPlayers = 0;}}
    if (WARNING_noPlayersToMove) {putText(camMask, "WARNING: no players to move", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_noPlayersToMove++; if(countWARNING_noPlayersToMove>10){WARNING_noPlayersToMove = false; countWARNING_noPlayersToMove = 0;}}
    if (WARNING_noPlayersToRemove) {putText(camMask, "WARNING: no players to remove", Point(cam.cols/8, cam.rows*4.0/9.0), FONT_HERSHEY_PLAIN, 5, Scalar(0,0,255), 4); countWARNING_noPlayersToRemove++; if(countWARNING_noPlayersToRemove>10){WARNING_noPlayersToRemove = false; countWARNING_noPlayersToRemove = 0;}}
    
    // imshow camera
    add(camMask, camMaskPositive, camMask);
    subtract(camMask, camMaskNegative, camMask);
    imshow("Camera", camMask);
}

void UserInputCallBackFunc( int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && !inputPressed) {
        inputPressed = true;
        if (inputState == "initialize") {
            for (int i=0; i<pointsSide.size(); i++) {
                if (pointsSide[i] == Point2f(-10,-10)) {pointsSide[i] = Point2f(x,y); ptrPlayer = &pointsSide[i]; break;}
            }
        }
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
        else if (inputState == "ball +/-") {
            if (ball == Point2f(-10,-10)) {ball = Point2f(x,y); ptrPlayer = &ball;}
            else {ball = Point2f(-10,-10);}
        }
        else if (inputState == "ball move") {
            ptrPlayer = &ball;
            *ptrPlayer = Point2f(x,y);
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
        drawUserInput();
    }
    else if (event == EVENT_MOUSEMOVE && inputPressed) {
        if (inputState == "initialize" || inputState == "away +" || inputState == "away move" || inputState == "home +" || inputState == "home move" || inputState == "ball +/-" || inputState == "ball move" || inputState == "optical flow") {
            if (ptrPlayer != nullptr) {*ptrPlayer = Point2f(x,y);}
        } else if (inputState == "sidelines move") {
            *ptrPlayer = Point2f(x,y);
            pointsCorner = poolSides2Corners(pointsSide);
        }
        drawUserInput();
    }
    else if (event == EVENT_LBUTTONUP && inputPressed) {
        inputPressed = false;
    }
}





// used to find players
vector<Point2f> getPlayerContours(Mat cam, Mat camHSVPrep) {
    
    vector<vector<Point>> contours;  //vector with more vectors inside
    vector<Vec4i> hierarchy;
    vector<Point2f> camPlayerLocations;

    findContours(camHSVPrep, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());     //variable that we are introducing to the vector
    string objectType;
    
    // Filter to remove noise
    for (int i=0; i < contours.size(); i++) {
        int area = contourArea(contours[i]);
        if (area>100) {                                      // POOL-SPECIFIC
            float peri = arcLength(contours[i], true);
            approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
            //drawContours(dst, conPoly, i, Scalar(0,0,0), FILLED);
            
            // find moments of the image
            Moments m = moments(contours[i]);
            camPlayerLocations.push_back(Point2f(m.m10/m.m00, m.m01/m.m00));
        }
    }
    return camPlayerLocations;
}

// using the binary pool image which has been HSV filtered and then preprocessed with a median blur, dilation, and erosion, find the contours of the pool -- these contours are near the boundary, but inside the pool
// if the contours form an area that is large enough, draw them as a solid black shape on (dst), which is a binary image of the Canny lines from the camera
// find the centroid of the detected pool area (hsvPoolCentroid)
void getContours(Mat cam, Mat camHSVPrep, Mat dst) {
    
    vector<vector<Point>> contours;  //vector with more vectors inside
    vector<Vec4i> hierarchy;

    findContours(camHSVPrep, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    vector<vector<Point>> conPoly(contours.size());
    vector<Rect> boundRect(contours.size());     //variable that we are introducing to the vector
    string objectType;
    
    // Filter to remove noise
    for (int i=0; i < contours.size(); i++) {
        int area = contourArea(contours[i]);
        if (area>500000) {                                      // POOL-SPECIFIC
            float peri = arcLength(contours[i], true);
            approxPolyDP(contours[i], conPoly[i], 0.02*peri, true);
            drawContours(dst, conPoly, i, Scalar(0,0,0), FILLED);
            
            // find moments of the image
            Moments m = moments(contours[i]);
            p = Point(m.m10/m.m00, m.m01/m.m00);
            // show the image with a point mark at the centroid
            //circle(cam, p, 10, Scalar(0,0,255), -1);
        }
    }
}

Mat colorTwist(Mat img) {
    // enables you to emphasize the colors of interest
    // performed worse when used to recognize the pool
    Mat newImg = Mat(img.rows, img.cols, img.type());
    for (int r=0; r<img.rows; r++) {
        for (int c=0; c<img.cols; c++) {
            Vec3b color = img.at<Vec3b>(Point(c,r));
            color[0] = (color[0]-33) * 255 / (255-33);
            color[1] = (color[1]-5) * 255 / (255-5);
            color[2] = (color[2]-1) * 255 / (255-1);
            newImg.at<Vec3b>(Point(c,r)) = color;
        }
    }
    return newImg;
}

Mat histogramEqualization(Mat bgr_image) {
    // automatically determines the best brightness and gain values, ideal for camera shots in poor lighting
    // analyzes the histogram of the individual color channels and luminance values, and from this distribution, computes relevant statistics such as the minimum, maximum, and average intensity values
    // simultaneously brightens some dark values and darkens some light values, while still using the full extent of the available dynamic range by flattening the histogram of the intensity
    
    // convert BGR color image to Lab
    Mat lab_image;
    cvtColor(bgr_image, lab_image, COLOR_BGR2Lab);
    
    // extract the L channel
    vector<Mat> lab_planes(3);
    split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]
    
    // apply the CLAHE algorithm to the L channel
    Ptr<CLAHE> clahe = createCLAHE();
    clahe->setClipLimit(4);
    Mat dst;
    clahe->apply(lab_planes[0], dst);

    // merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    merge(lab_planes, lab_image);

    // convert back to BGR
    Mat image_clahe;
    cvtColor(lab_image, image_clahe, COLOR_Lab2BGR);

    return image_clahe;
}

Mat drawAnimatedPool(vector<Point2f> cameraPoolCorners, vector<Point2f> cameraBoundaryPoints) { //, vector<Point2f> camPlayers) {
    Mat pool;
    
    // pool colors
    Scalar poolBlue = Scalar(255,255,204);
    Scalar poolYellow = Scalar(0,255,255);
    Scalar poolRed = Scalar(0,0,255);
    Scalar poolWhite = Scalar(255,255,255);
    Vec3b darkerTint = {50,50,50};
    
    // take measurements
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
    
    // find the homography that converts the camera plane to the animated pool plane and apply it to each point in (cameraBoundaryPoints)
    vector<Point2f> animatedPoolCorners = {Point2f(0,0),Point2f(windowLength,0),Point2f(windowLength,windowHeight),Point2f(0,windowHeight)};
    Mat H = findHomography(cameraPoolCorners,animatedPoolCorners);
    vector<Point2f> cameraBoundary;
    for (int i=0; i<cameraBoundaryPoints.size(); i++) {
        float x = cameraBoundaryPoints[i].x*H.at<double>(0,0) + cameraBoundaryPoints[i].y*H.at<double>(0,1) + H.at<double>(0,2);
        float y = cameraBoundaryPoints[i].x*H.at<double>(1,0) + cameraBoundaryPoints[i].y*H.at<double>(1,1) + H.at<double>(1,2);
        float z = cameraBoundaryPoints[i].x*H.at<double>(2,0) + cameraBoundaryPoints[i].y*H.at<double>(2,1) + H.at<double>(2,2);
        float xVal, yVal;
        xVal = x/z; yVal = y/z;
        cameraBoundary.push_back(Point2f(xVal,yVal));
    }
    
    for (int i=0; i<cameraBoundaryPoints.size(); i++) {
        float x = cameraBoundaryPoints[i].x*H.at<double>(0,0) + cameraBoundaryPoints[i].y*H.at<double>(0,1) + H.at<double>(0,2);
        float y = cameraBoundaryPoints[i].x*H.at<double>(1,0) + cameraBoundaryPoints[i].y*H.at<double>(1,1) + H.at<double>(1,2);
        float z = cameraBoundaryPoints[i].x*H.at<double>(2,0) + cameraBoundaryPoints[i].y*H.at<double>(2,1) + H.at<double>(2,2);
        float xVal, yVal;
        //if (x/z > windowLength) {xVal = windowLength;}
        //else if (x/z < 0) {xVal = 0;}
        //else (xVal = x/z);
        //if (y/z > windowHeight) {yVal = windowHeight;}
        //else if (y/z < 0) {yVal = 0;}
        //else {yVal = y/z;}
        xVal = x/z; yVal = y/z;
        cameraBoundary.push_back(Point2f(xVal,yVal));
    }

    for (int i=0; i<cameraBoundary.size(); i++) {
        if (cameraBoundary[i].x > windowLength) {
            double x1 = windowLength;
            double m1 = (cameraBoundary[i].y - cameraBoundary[(i-1)%cameraBoundary.size()].y) / (cameraBoundary[i].x - cameraBoundary[(i-1)%cameraBoundary.size()].x);
            double y1 = cameraBoundary[i].y + m1*(x1-cameraBoundary[i].x);
            double x2 = windowLength;
            double m2 = (cameraBoundary[i].y - cameraBoundary[(i+1)%cameraBoundary.size()].y) / (cameraBoundary[i].x - cameraBoundary[(i+1)%cameraBoundary.size()].x);
            double y2 = cameraBoundary[i].y + m2*(x2-cameraBoundary[i].x);
            cameraBoundary[i] = Point2f(x1,y1);
            cameraBoundary.push_back(cameraBoundary.back());
            for (int j=static_cast<int>(cameraBoundary.size()-2); j>i+1; j--) {
                cameraBoundary[j] = cameraBoundary[j-1];
            }
            cameraBoundary[i+1] = Point2f(x2,y2);
        } else if (cameraBoundary[i].y > windowHeight) {
            cameraBoundary[i] = Point2f(cameraBoundary[i].x,windowHeight);
        } /*else if (cameraBoundary[i].y > windowHeight) {
            double y1 = windowHeight;
            double m1 = (cameraBoundary[i].y - cameraBoundary[(i-1)%cameraBoundary.size()].y) / (cameraBoundary[i].x - cameraBoundary[(i-1)%cameraBoundary.size()].x);
            double x1 = cameraBoundary[i].x - (cameraBoundary[i].y-y1)/m1;
            double y2 = windowHeight;
            double m2 = (cameraBoundary[i].y - cameraBoundary[(i+1)%cameraBoundary.size()].y) / (cameraBoundary[i].x - cameraBoundary[(i+1)%cameraBoundary.size()].x);
            double x2 = cameraBoundary[i].x - (cameraBoundary[i].y-y2)/m2;
            cameraBoundary[i] = Point2f(x1,y1);
            cameraBoundary.push_back(cameraBoundary.back());
            for (int j=static_cast<int>(cameraBoundary.size()-2); j>i+1; j--) {
                cameraBoundary[j] = cameraBoundary[j-1];
            }
            cameraBoundary[i+1] = Point2f(x2,y2);
        }*/
    }
    
    // find centroid of the animated pool window and the lines of poolBoundary
    Point2f boundaryMax = Point2f(0,0);
    Point2f boundaryMin = Point2f(windowLength,windowHeight);
    vector<vector<double>> poolBoundaryLines;
    for (int i=0; i<cameraBoundary.size(); i++) {
        if (cameraBoundary[i].x > boundaryMax.x) {boundaryMax.x = cameraBoundary[i].x;}
        if (cameraBoundary[i].x < boundaryMin.x) {boundaryMin.x = cameraBoundary[i].x;}
        if (cameraBoundary[i].y > boundaryMax.y) {boundaryMax.y = cameraBoundary[i].y;}
        if (cameraBoundary[i].y < boundaryMin.y) {boundaryMin.y = cameraBoundary[i].y;}
        
        double m = (cameraBoundary[i].y - cameraBoundary[(i+1)%cameraBoundary.size()].y) / (cameraBoundary[i].x - cameraBoundary[(i+1)%cameraBoundary.size()].x);
        double b = cameraBoundary[i].y - m*cameraBoundary[i].x;
        poolBoundaryLines.push_back({m,b});
    }
    Point2f animCentroid = Point2f(windowLength/2,windowHeight/2);//Point2f((boundaryMax.x+boundaryMin.x)/2, (boundaryMax.y+boundaryMin.y)/2);
    
    // sort points in (cameraBoundary) by their angle around the centroid
    /*Point stationaryPt = cameraBoundary[0];
    for (int i=1; i<cameraBoundary.size(); i++) {
        for (int j=static_cast<int>(cameraBoundary.size()-1); j>i; j--) {
            double angle1 = atan2(stationaryPt.y-cameraBoundary[j-1].y, stationaryPt.x-cameraBoundary[j-1].x);
            double angle2 = atan2(stationaryPt.y-cameraBoundary[j].y, stationaryPt.x-cameraBoundary[j].x);
            if (angle2 < angle1) {
                Point2f minValue = cameraBoundary[j];
                cameraBoundary[j] = cameraBoundary[j-1];
                cameraBoundary[j-1] = minValue;
            }
        }
    }*/
    
    // draw on the canvas
    pool.create(windowHeight, windowLength, CV_8UC3);
    pool.setTo(poolBlue);                      // create a blank pool canvas
    line(pool, Point(0,windowHeight/2-windowHeight*goalWidth/poolWidth/2), Point(0,windowHeight/2+windowHeight*goalWidth/poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength,windowHeight/2-windowHeight*goalWidth/poolWidth/2), Point(windowLength,windowHeight/2+windowHeight*goalWidth/poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength*2/poolLength,0), Point(windowLength*2/poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength-windowLength*2/poolLength,0), Point(windowLength-windowLength*2/poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength*6/poolLength,0), Point(windowLength*6/poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength-windowLength*6/poolLength,0), Point(windowLength-windowLength*6/poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength*half/poolLength,0), Point(windowLength*half/poolLength,windowHeight), poolWhite,3);      // half pool
    
    // draw a line connecting the area of the pool that can be seen by the camera
    for (int i=0; i<cameraBoundary.size()-1; i++) {
        line(pool, Point2f(cameraBoundary[i].x,cameraBoundary[i].y), Point2f(cameraBoundary[(i+1)%cameraBoundary.size()].x,cameraBoundary[(i+1)%cameraBoundary.size()].y), Scalar(255,0,0), 2.5);
    }
    Mat poolMask1 = Mat(pool.rows, pool.cols, CV_8UC3);
    inRange(pool, Scalar(255,0,0), Scalar(255,0,0), poolMask1);

    // darken the part of the pool that is not in the field of view of the camera
    for (double col=0; col<windowLength; col++) {
        for (double row=0; row<windowHeight; row++) {
            
        // for each point in (poolBoundary), iterate through all of the slope-intercept pairs in (lines) to determine if the point is on the same side of the line as the centroid; if yes store the value 1 in (pointChecker), if not store the value 0
            for (int j=0; j<poolBoundaryLines.size(); j++) {
                double centVal = animCentroid.y - poolBoundaryLines[j][0]*animCentroid.x;
                double pointVal = row - poolBoundaryLines[j][0]*col;
                Vec3b poolBGR = pool.at<Vec3b>(Point(col,row));
                if (abs(poolBoundaryLines[j][0]*col+poolBoundaryLines[j][1] - row) < 1) {
                    continue;
                } else if (abs(poolBoundaryLines[j][0]) > 1000) {
                    if (col == cameraBoundary[j].x) {
                        continue;
                    } else if (cameraBoundary[j].x > animCentroid.x && cameraBoundary[j].x > col) {
                        continue;
                    } else if (cameraBoundary[j].x < animCentroid.x && cameraBoundary[j].x < col) {
                        continue;
                    } else {
                        poolBGR -= darkerTint;
                        pool.at<Vec3b>(Point(col,row)) = poolBGR;
                        //cout<<"point"<<": ("<<col<<", "<<row<<")   animatedPoint.x = "<<cameraBoundary[j].x<<"   centroid.x = "<<animCentroid.x<<endl;                                                    // DEBUG MODE
                        break;
                    }
                } else if (pointVal<poolBoundaryLines[j][1] && centVal>poolBoundaryLines[j][1]) {
                    poolBGR -= darkerTint;
                    pool.at<Vec3b>(Point(col,row)) = poolBGR;
                    //cout<<"point"<<": ("<<col<<", "<<row<<")   "<<pointVal<<" > "<<poolBoundaryLines[j][1]<<" and "<<centVal<<" < "<<poolBoundaryLines[j][1]<<"   line: "<<poolBoundaryLines[j][0]<<", "<<poolBoundaryLines[j][1]<<"   poolBoundary: ("<<cameraBoundary[j].x<<", "<<cameraBoundary[j].y<<")"<<endl;                   // DEBUG MODE
                    break;
                } else if (pointVal>poolBoundaryLines[j][1] && centVal<poolBoundaryLines[j][1]) {
                    poolBGR -= darkerTint;
                    pool.at<Vec3b>(Point(col,row)) = poolBGR;
                    //cout<<"point"<<": ("<<col<<", "<<row<<")   "<<pointVal<<" > "<<poolBoundaryLines[j][1]<<" and "<<centVal<<" < "<<poolBoundaryLines[j][1]<<"   line: "<<poolBoundaryLines[j][0]<<", "<<poolBoundaryLines[j][1]<<"   poolBoundary: ("<<cameraBoundary[j].x<<", "<<cameraBoundary[j].y<<")"<<endl;                   // DEBUG MODE
                    break;
                }
            }
            /*//Vec3b poolMaskBGR = poolMask3.at<Vec3b>(Point(col,row));
            if (poolMaskBGR[0]==0) {
                Vec3b poolBGR = pool.at<Vec3b>(Point(col,row));
                poolBGR -= darkerTint;
                pool.at<Vec3b>(Point(col,row)) = poolBGR;
            }*/
        }
    }
    
    /*if (camPlayers.size()) {
        for (int i=0; i<camPlayers.size(); i++) {
            float x = camPlayers[i].x*H.at<double>(0,0) + camPlayers[i].y*H.at<double>(0,1) + H.at<double>(0,2);
            float y = camPlayers[i].x*H.at<double>(1,0) + camPlayers[i].y*H.at<double>(1,1) + H.at<double>(1,2);
            float z = camPlayers[i].x*H.at<double>(2,0) + camPlayers[i].y*H.at<double>(2,1) + H.at<double>(2,2);
            float xVal, yVal;
            xVal = x/z; yVal = y/z;
            if (xVal>=0 && xVal<=windowLength && yVal>=0 && yVal<=windowHeight)
            {
                Vec3b poolBGR = pool.at<Vec3b>(Point(static_cast<int>(xVal),static_cast<int>(yVal)));
                if (poolBGR[0] == 255 || poolBGR[0] == 0) {
                    circle(pool, Point(xVal,yVal), 3, Scalar(0,120,0), FILLED);
                }
            }
        }
    }*/
    
    // show animated pool
    //imshow("animated pool", pool);                                                            // DEBUG MODE
    //imshow("poolMask1: inrange filtered", poolMask1);                                         // DEBUG MODE
    //imshow("poolMask2: contours with filling", poolMask2);                                    // DEBUG MODE
    //imshow("shaded white BGR", poolMask3);                                                    // DEBUG MODE
    //imshow("poolMask4: contours without filling", poolMask4);                                 // DEBUG MODE
    
    return pool;
}



void LocationCallBackFunc(int event, int x, int y, int flags, void* userdata) {
    
    if ( event == EVENT_MOUSEMOVE) {
        cout<<"mouse: ("<<x<<", "<<y<<")"<<endl;
    }
}

void OpticalFlowCallBackFunc(int event, int x, int y, int flags, void* userdata) {
    
    if ( event == EVENT_LBUTTONDOWN) {
        cout<<"mouse: ("<<x<<", "<<y<<")"<<endl;
        p0.push_back(Point2f(x,y));
    } else if ( event == EVENT_RBUTTONDOWN) {
        temporaryPoolCorners.push_back(Point2f(x,y));
    }
}

void UserPoolIdentifierCallBackFunc(int event, int x, int y, int flags, void* userdata) {
    
    if ( event == EVENT_LBUTTONDOWN) {
        //cout<<"I AM CLICKING !!! buttonPressed is equal to "<<buttonPressed<<endl;
        if (buttonPressed == 0) {
            for (int i=0; i<points2M.size(); i++) {
                if (sqrt(pow(x-points2M[i].x,2) + pow(y-points2M[i].y,2))<5) {
                    moving = true;
                    moverVariable = i;
                }
            }
            if (!doneInitializing) {
                points2M.push_back(Point2f(x,y));
                circle(frame, Point(x,y), 10, Scalar(0,0,255), FILLED);
            }
        } else if (buttonPressed == 1) {
            for (int i=0; i<points5M.size(); i++) {
                if (sqrt(pow(x-points5M[i].x,2) + pow(y-points5M[i].y,2))<5) {
                    moving = true;
                    moverVariable = i;
                }
            }
            if (!doneInitializing) {
            points5M.push_back(Point2f(x,y));
            circle(frame, Point(x,y), 10, Scalar(255,255,0), FILLED);
            }
        } else if (buttonPressed == 2) {
            for (int i=0; i<pointsHalf.size(); i++) {
                if (sqrt(pow(x-pointsHalf[i].x,2) + pow(y-pointsHalf[i].y,2))<5) {
                    moving = true;
                    moverVariable = i;
                }
            }
            if (!doneInitializing) {
            pointsHalf.push_back(Point2f(x,y));
            circle(frame, Point(x,y), 10, Scalar(255,255,255), FILLED);
            }
        } else if (buttonPressed == 3) {
            for (int i=0; i<pointsCorner.size(); i++) {
                if (sqrt(pow(x-pointsCorner[i].x,2) + pow(y-pointsCorner[i].y,2))<5) {
                    moving = true;
                    moverVariable = i;
                }
            }
            if (!doneInitializing) {
            pointsCorner.push_back(Point2f(x,y));
            circle(frame, Point(x,y), 10, Scalar(0,0,0), FILLED);
            }
        } else if (buttonPressed == 4) {
            for (int i=0; i<pointsSide.size(); i++) {
                if (sqrt(pow(x-pointsSide[i].x,2) + pow(y-pointsSide[i].y,2))<5) {
                    moving = true;
                    moverVariable = i;
                }
            }
            if (!doneInitializing) {
            pointsSide.push_back(Point2f(x,y));
            circle(frame, Point(x,y), 5, Scalar(0,0,0), FILLED);
            }
        }
    }
    
    if (event == EVENT_MOUSEMOVE && moving) {
        if (buttonPressed == 0) {
            points2M[moverVariable] = Point2f(x,y);
        } else if (buttonPressed == 1) {
            points5M[moverVariable] = Point2f(x,y);
        } else if (buttonPressed == 2) {
            pointsHalf[moverVariable] = Point2f(x,y);
        } else if (buttonPressed == 3) {
            pointsCorner[moverVariable] = Point2f(x,y);
        } else if (buttonPressed == 4) {
            pointsSide[moverVariable] = Point2f(x,y);
        }
        
        for (int i=0; i<pointsSide.size(); i++) {
            circle(frame, pointsSide[i], 5, Scalar(0,0,0), -1);
        }
        imshow("Camera",frame);
    }
    
    if (event == EVENT_LBUTTONUP) {
        moving = false;
    }
    
}

void UserPoolStateCallBackFunc(int event, int x, int y, int flags, void* userdata) {
    if ( event == EVENT_LBUTTONDOWN) {
        if (y<100) {state2M = true; state5M = false; stateHalf = false; stateCorner = false; stateSide = false; buttonPressed = 0; moving = false;}
        else if (y<200) {state2M = false; state5M = true; stateHalf = false; stateCorner = false; stateSide = false; buttonPressed = 1; moving = false;}
        else if (y<300) {state2M = false; state5M = false; stateHalf = true; stateCorner = false; stateSide = false; buttonPressed = 2; moving = false;}
        else if (y<400) {state2M = false; state5M = false; stateHalf = false; stateCorner = true; stateSide = false; buttonPressed = 3; moving = false;}
        else if (y<500) {state2M = false; state5M = false; stateHalf = false; stateCorner = false; stateSide = true; buttonPressed = 4; moving = false;}
    }
}

void findPoolCornersFromUserInput(Mat img) {
    // need to translate a 5, half, 5 indication to predicted pool corner positions
    // need 8 points, where each corner counts as 2 points, and a pair of points which are at specific positions count as 4 points
    // start by requiring the two corners on one side, and 5-half-5 on the other...can we find the homography knowing 5-half-5 and where the corners will be?
    
    /*vector<Point2f> userPoints;
    for (int i=0; i<points2M.size(); i++) {
        userPoints.push_back(points2M[i]);
    }
    for (int i=0; i<points5M.size(); i++) {
        userPoints.push_back(points5M[i]);
    }
    for (int i=0; i<pointsHalf.size(); i++) {
        userPoints.push_back(pointsHalf[i]);
    }
    for (int i=0; i<pointsCorner.size(); i++) {
        userPoints.push_back(pointsCorner[i]);
    }
    for (int i=0; i<pointsSide.size(); i++) {
        userPoints.push_back(pointsSide[i]);
    }
    Point2f average = Point2f(0.0, 0.0);
    for (int i=0; i<userPoints.size(); i++) {
        average.x += userPoints[i].x;
        average.y += userPoints[i].y;
    }
    average.x = average.x / userPoints.size();
    average.y = average.y / userPoints.size();
    // sort each point in (userPoints) by its angle around the centroid (average)
    for (int i=0; i<userPoints.size(); i++) {
        for (int j=static_cast<int>(userPoints.size()-1); j>i; j--) {
            double centroidAngle1 = atan2(average.y-userPoints[j-1].y, average.x-userPoints[j-1].x);
            double centroidAngle2 = atan2(average.y-userPoints[j].y, average.x-userPoints[j].x);
            if (centroidAngle2 < centroidAngle1) {
                Point2d minValue = userPoints[j];
                userPoints[j] = userPoints[j-1];
                userPoints[j-1] = minValue;
            }
        }
    }
    
    int startIterator = 0;
    for (int i=0; i<userPoints.size(); i++) {
        for (int j=0; j<pointsCorner.size(); j++) {
            if (userPoints[i] == pointsCorner[j]) {
                startIterator = i;
            }
        }
    }*/
    vector<vector<double>> intersectionPoints;
    double m, b, x1, y1, x2, y2;
    for (int i=0; i<4; i++) {
        x1 = pointsSide[i*2].x; y1 = pointsSide[i*2].y;
        x2 = pointsSide[i*2+1].x; y2 = pointsSide[i*2+1].y;
        m = static_cast<double>(y2-y1)/(x2-x1);
        b = y1 - m*x1;
        intersectionPoints.push_back({m,b});
    }
    
    // create intersection points (points) from the vector of slope-intercept values (intersectionPoints)
    cornerPoints.clear();
    for (int i=0; i<4; i++) {
        double x = static_cast<double>((intersectionPoints[(i+1)%4][1] - intersectionPoints[i][1]))/(intersectionPoints[i][0]-intersectionPoints[(i+1)%4][0]);
        double y = intersectionPoints[i][0]*x + intersectionPoints[i][1];
        cornerPoints.push_back(Point2f(x,y));
        
    }
    
    // sort each point in (cornerPoints) by its angle around the centroid (average)
    Point2f average = Point2f(0.0, 0.0);
    for (int i=0; i<cornerPoints.size(); i++) {
        average.x += cornerPoints[i].x;
        average.y += cornerPoints[i].y;
    }
    average.x = average.x / cornerPoints.size();
    average.y = average.y / cornerPoints.size();
    for (int i=0; i<cornerPoints.size(); i++) {
        for (int j=static_cast<int>(cornerPoints.size()-1); j>i; j--) {
            double centroidAngle1 = atan2(average.y-cornerPoints[j-1].y, average.x-cornerPoints[j-1].x);
            double centroidAngle2 = atan2(average.y-cornerPoints[j].y, average.x-cornerPoints[j].x);
            if (centroidAngle2 < centroidAngle1) {
                Point2d minValue = cornerPoints[j];
                cornerPoints[j] = cornerPoints[j-1];
                cornerPoints[j-1] = minValue;
            }
        }
    }
    if (cornerPoints.size()==4) {
        Point2f holder1ForCameraPoolCorners = cornerPoints[0];
        Point2f holder2ForCameraPoolCorners = cornerPoints[1];
        cornerPoints[0] = cornerPoints[2];
        cornerPoints[1] = cornerPoints[3];
        cornerPoints[2] = holder1ForCameraPoolCorners;
        cornerPoints[3] = holder2ForCameraPoolCorners;
    }
    
    //for (int i=0; i<4; i++) {
    //    circle(img, cornerPoints[i], 20, Scalar(0,0,0), FILLED);
    //    putText(img, to_string(i), cornerPoints[i], FONT_HERSHEY_PLAIN, 5, Scalar(0,0,0),3);
    //    line(img, cornerPoints[i], cornerPoints[(i+1)%4], Scalar(0,0,255), 3);
    //}
    temporaryPoolCorners = cornerPoints;
}

double distancePoint2Line(Point pt, Point ln1_1, Point ln1_2) {
    double m1 = static_cast<double>(ln1_1.y-ln1_2.y)/(ln1_1.x-ln1_2.x);
    double m2 = -1/m1;
    double b1 = ln1_1.y - m1 * ln1_1.x;
    double b2 = pt.y - m2 * pt.x;
    
    double x = (b2-b1)/(m1-m2);
    double y = m1*x + b1;
    
    double dist = sqrt(pow(x-pt.x,2)+pow(y-pt.y,2));
    
    return dist;
}

Vec4i drawLongLine(Mat img, Vec4i points) {
    double m, b, x1, y1, x2, y2;
    x1 = points[0]; y1 = points[1];
    x2 = points[2]; y2 = points[3];
    m = static_cast<double>(y2-y1)/(x2-x1);
    b = y1 - m*x1;
    
    //if (m>100 || m<-100) {line(img, Point(x1,y1), Point(x2,y2), Scalar(255,0,0),2);}          // DEBUG MODE
    //else {line(img, Point(0,b), Point(img.cols,m*img.cols+b), Scalar(255,0,0), 2);}           // DEBUG MODE
    poolLines.push_back({m,b});
    
    return {static_cast<int>(x1),static_cast<int>(y1),static_cast<int>(x2),static_cast<int>(y2)};
}

vector<Point2f> findCamPoolBoundary(Mat img) {
    
    // add the slope-intercept values of the camera border to the vector (poolLines)
    poolLines.push_back({0.0,0.0});
    poolLines.push_back({-100.0,100.0*img.cols});
    poolLines.push_back({0.0,static_cast<double>(img.rows)});
    poolLines.push_back({100.0,0.0});
    
    // create intersection points (points) from the vector of slope-intercept values (poolLines)
    vector<Point2f> points;
    for (int i=0; i<poolLines.size(); i++) {
        for (int j=i+1; j<poolLines.size(); j++) {
            double x = static_cast<double>((poolLines[j][1] - poolLines[i][1]))/(poolLines[i][0]-poolLines[j][0]);
            double y = poolLines[i][0]*x + poolLines[i][1];
            if (x>=0 && x<=img.cols && y>=0 && y<=img.rows) {
                points.push_back(Point2f(x,y));
            }
        }
    }
    if (points.size() == 0) {
        points.push_back(Point(0,0));
        points.push_back(Point(img.cols,0));
        points.push_back(Point(img.cols,img.rows));
        points.push_back(Point(0,img.rows));
    }
    
    // sort each point in (points) by its angle around the centroid (p)
    for (int i=0; i<points.size(); i++) {
        for (int j=static_cast<int>(points.size()-1); j>i; j--) {
            double centroidAngle1 = atan2(p.y-points[j-1].y, p.x-points[j-1].x);
            double centroidAngle2 = atan2(p.y-points[j].y, p.x-points[j].x);
            if (centroidAngle2 < centroidAngle1) {
                Point2d minValue = points[j];
                points[j] = points[j-1];
                points[j-1] = minValue;
            }
        }
    }
    
    /*// place an identifier on each point in (points) with a circle to make sure it is sorted correctly        // DEBUG MODE*
    for (int i=0; i<points.size(); i++) {
        circle(img, points[i], 10, Scalar(0,255,0), -1);
        int x; int y;
        if (points[i].x > img.cols - 15) {x = points[i].x - 150;}
        else {x = points[i].x;}
        if (points[i].y < 15) {y = points[i].y + 150;}
        else {y = points[i].y;}
        //putText(img, to_string(i), Point(x,y), FONT_HERSHEY_PLAIN, 5, Scalar(0,255,0), 3);
    }*/
    
    // for each point in (points), iterate through all of the slope-intercept pairs in (lines) to determine if the point is on the same side of the line as the centroid; if yes store the value 1 in (pointChecker), if not store the value 0
    vector<bool> pointChecker (points.size(), 1);
    for (int i=0; i<points.size(); i++) {
        for (int j=0; j<poolLines.size(); j++) {
            double centVal = -1*poolLines[j][0]*p.x + p.y;
            double pointVal = -1*poolLines[j][0]*points[i].x + points[i].y;
            if (abs(poolLines[j][0]*points[i].x+poolLines[j][1] - points[i].y) < 5) {continue;}
            else if (pointVal<poolLines[j][1] && centVal>poolLines[j][1]) {
                pointChecker[i] = 0;
                //cout<<"point "<<i<<": ("<<points[i].x<<", "<<points[i].y<<")   "<<pointVal<<" < "<<lines[j][1]<<" and "<<centVal<<" > "<<lines[j][1]<<"   line: "<<lines[j][0]<<", "<<lines[j][1]<<"   centroid: ("<<p.x<<", "<<p.y<<")"<<endl;
                break;
            }
            else if (pointVal>poolLines[j][1] && centVal<poolLines[j][1]) {
                pointChecker[i] = 0;
                //cout<<"point "<<i<<": ("<<points[i].x<<", "<<points[i].y<<")   "<<pointVal<<" > "<<lines[j][1]<<" and "<<centVal<<" < "<<lines[j][1]<<"   line: "<<lines[j][0]<<", "<<lines[j][1]<<"   centroid: ("<<p.x<<", "<<p.y<<")"<<endl;
                break;
            }
        }
    }
    vector<Point2f> poolBoundary;
    for (int i=0; i<points.size(); i++) {
        if (pointChecker[i] == 1) {
            poolBoundary.push_back(points[i]);
        }
    }
    
    // for each point in (points) which corresponds to a 1 in (pointChecker), plot it and connect it to the other points
    if (poolBoundary.size()) {
        for (int i=0; i<poolBoundary.size()-1; i++) {
            line(img, poolBoundary[i], poolBoundary[i+1], Scalar(255,0,255), 2);
        }
        line(img, poolBoundary.back(), poolBoundary.front(), Scalar(255,0,255), 2);
    }
    
    return poolBoundary;
}

vector<Point2f> findCamPoolCorners(Mat img, vector<Point2f> poolBoundary) {
    
    // make sure the points in (poolBoundary) are correct                             // DEBUG MODE*
    /*for (int i=0; i<poolBoundary.size(); i++) {
        int x; int y;
        if (poolBoundary[i].x > img.cols - 15) {x = poolBoundary[i].x - 150;}
        else {x = poolBoundary[i].x;}
        if (poolBoundary[i].y < 15) {y = poolBoundary[i].y + 150;}
        else {y = poolBoundary[i].y;}
        circle(img, Point(poolBoundary[i].x,poolBoundary[i].y), 5, Scalar(0,255,0));
        putText(img, to_string(i), Point(x,y), FONT_HERSHEY_PLAIN, 10, Scalar(0,255,0), 3);
    }*/
    
    // find all the points which are not near the edge because these must be pool corners and mark them with big purple circles
    vector<Point2f> cameraPoolCorners;
    if (poolBoundary.size()) {
        for (int i=0; i<poolBoundary.size(); i++) {
            bool sideChecker = 0;
            for (unsigned long j=poolLines.size()-1; j>poolLines.size()-5; j--) {
                if (abs(poolLines[j][0]*poolBoundary[i].x+poolLines[j][1] - poolBoundary[i].y) < 5) {
                    sideChecker = 1;
                    break;
                }
            }
            if (sideChecker == 0) {cameraPoolCorners.push_back(poolBoundary[i]);}
        }
    }
    for (int i=0; i<cameraPoolCorners.size(); i++) {                              // DEBUG MODE*
        circle(img, cameraPoolCorners[i], 20, Scalar(150,0,150), FILLED);
    }
    
    // eliminate the pool corners that are not probable
    double difPoolCorners = INT_MAX;
    Point2f theCamPoolCorner;
    if (lastPoolCorners.size() && cameraPoolCorners.size()>1) {
        for (int i=0; i<cameraPoolCorners.size(); i++) {
            for (int j=0; j<lastPoolCorners.size(); j++) {
                double distance = sqrt(pow((lastPoolCorners[j].x-cameraPoolCorners[i].x),2)*pow((lastPoolCorners[j].y-cameraPoolCorners[i].y),2));
                if (distance < difPoolCorners) {difPoolCorners = distance; theCamPoolCorner = cameraPoolCorners[i];}
            }
        }
        cameraPoolCorners.clear();
        cameraPoolCorners.push_back(theCamPoolCorner);
        circle(img, cameraPoolCorners[0], 20, Scalar(0,255,0));
    }
    
    // interpolate the four pool corners if the camera were to be infinitely large; case 1 assume there is only 1 corner; case 2 assume there are 2 corners next to each other--this has not been implemented yet
    double topPixels = 1475;                // POOL-SPECIFIC
    double rightPixels = 1000;              // POOL-SPECIFIC
    double bottomPixels = 3500;             // POOL-SPECIFIC
    double leftPixels = 1160;               // POOL-SPECIFIC
    double pixelLength1, pixelLength2, pixelLength3, pixelLength4;
    
    for (int i=0; i<poolBoundary.size(); i++) {
        //for (int j=0; j<cameraPoolCorners.size(); j++) {
            if (cameraPoolCorners.size() && cameraPoolCorners[0] == poolBoundary[i]) {
                //circle(img, poolBoundary[i], 20, Scalar(150,0,150), FILLED);                              // DEBUG MODE
                //circle(img, poolBoundary[(i-1)%poolBoundary.size()], 20, Scalar(150,0,0), FILLED);        // DEBUG MODE
                //circle(img, poolBoundary[(i+1)%poolBoundary.size()], 20, Scalar(0,0,150), FILLED);        // DEBUG MODE
                double centroidAngle = atan2(poolBoundary[i].y-p.y, poolBoundary[i].x-p.x);        // note: atan2 returns [-pi, pi]
                double theta1 = atan2(poolBoundary[(i-1)%poolBoundary.size()].y-poolBoundary[i].y, poolBoundary[(i-1)%poolBoundary.size()].x-poolBoundary[i].x);
                double theta2 = atan2(poolBoundary[(i+1)%poolBoundary.size()].y-poolBoundary[i].y, poolBoundary[(i+1)%poolBoundary.size()].x-poolBoundary[i].x);
                if (centroidAngle < -M_PI_2) {                           // top left
                    pixelLength1 = leftPixels;
                    pixelLength2 = topPixels;
                    pixelLength3 = rightPixels;
                    pixelLength4 = bottomPixels;
                    //cout<<"top left "<<centroidAngle/M_PI<<endl;
                } else if (centroidAngle < 0) {                         // top right
                    pixelLength1 = topPixels;
                    pixelLength2 = rightPixels;
                    pixelLength3 = bottomPixels;
                    pixelLength4 = leftPixels;
                    //cout<<"top right "<<centroidAngle/M_PI<<endl;
                } else if (centroidAngle < M_PI_2) {                   // bottom right
                    pixelLength1 = rightPixels;
                    pixelLength2 = bottomPixels;
                    pixelLength3 = leftPixels;
                    pixelLength4 = topPixels;
                    //cout<<"bottom right "<<centroidAngle/M_PI<<endl;
                } else {                                                // bottom left
                    pixelLength1 = bottomPixels;
                    pixelLength2 = leftPixels;
                    pixelLength3 = topPixels;
                    pixelLength4 = rightPixels;
                    //cout<<"bottom left "<<centroidAngle/M_PI<<endl;
                }
                //circle(img, p, 10, Scalar(0,0,100), FILLED);
                double x1 = poolBoundary[i].x + pixelLength1*cos(theta1);
                double y1 = poolBoundary[i].y + pixelLength1*sin(theta1);
                double x2 = poolBoundary[i].x + pixelLength2*cos(theta2);
                double y2 = poolBoundary[i].y + pixelLength2*sin(theta2);
                double x3, y3;
                
                if (poolBoundary.size() == 4) {
                    x3 = poolBoundary[(i+2)%poolBoundary.size()].x;
                    y3 = poolBoundary[(i+2)%poolBoundary.size()].x;
                } else {
                    vector<double> line1, line2;
                    double theta3 = atan2(poolBoundary[(i+3)%poolBoundary.size()].y-y2, poolBoundary[(i+3)%poolBoundary.size()].x-x2);
                    double theta4 = atan2(poolBoundary[(i-3)%poolBoundary.size()].y-y1, poolBoundary[(i-3)%poolBoundary.size()].x-x1);
                    int pointsThroughLine3 = 0;
                    int pointsThroughLine4 = 0;
                    //if (x1>=0 && x1<=img.cols && y1>=0 && y1<=img.rows) {pointsThroughLine4++;}
                    //if (x2>=0 && x2<=img.cols && y2>=0 && y2<=img.rows) {pointsThroughLine3++;}
                    for (int k=0; k<poolBoundary.size(); k++) {
                        if (abs(theta3*(poolBoundary[k].x-x2)-poolBoundary[k].y+y2)<0.1) {pointsThroughLine3++;}
                        else if (abs(theta4*(poolBoundary[k].x-x1)-poolBoundary[k].y+y1)<0.1) {pointsThroughLine4++;}
                    }
                    if (pointsThroughLine3 > pointsThroughLine4) {
                        x3 = x2 + pixelLength3*cos(theta3);
                        y3 = y2 + pixelLength3*sin(theta3);
                    } else {
                        x3 = x1 + pixelLength4*cos(theta4);
                        y3 = y1 + pixelLength4*sin(theta4);
                    }
                }
                cameraPoolCorners.push_back(Point2f(x2,y2));
                cameraPoolCorners.push_back(Point2f(x3,y3));
                cameraPoolCorners.push_back(Point2f(x1,y1));
                lastPoolCorners.clear();
                lastPoolCorners = cameraPoolCorners;
                break;
            }
        //}
    }
    
    // sort each point in (cameraPoolCorners) by its angle around the centroid (p), making the top left point 0 and ascending in the clockwise direction
    for (int i=0; i<cameraPoolCorners.size(); i++) {
        for (int j=static_cast<int>(cameraPoolCorners.size()-1); j>i; j--) {
            double centroidAngle1 = atan2(p.y-cameraPoolCorners[j-1].y, p.x-cameraPoolCorners[j-1].x);
            double centroidAngle2 = atan2(p.y-cameraPoolCorners[j].y, p.x-cameraPoolCorners[j].x);
            if (centroidAngle2 < centroidAngle1) {
                Point2d minValue = cameraPoolCorners[j];
                cameraPoolCorners[j] = cameraPoolCorners[j-1];
                cameraPoolCorners[j-1] = minValue;
            }
        }
    }
    if (cameraPoolCorners.size()==4) {
        Point2f holder1ForCameraPoolCorners = cameraPoolCorners[0];
        Point2f holder2ForCameraPoolCorners = cameraPoolCorners[1];
        cameraPoolCorners[0] = cameraPoolCorners[2];
        cameraPoolCorners[1] = cameraPoolCorners[3];
        cameraPoolCorners[2] = holder1ForCameraPoolCorners;
        cameraPoolCorners[3] = holder2ForCameraPoolCorners;
    }
    
    // check if the cameraPoolCorners make sense
    for (int i=0; i<cameraPoolCorners.size(); i++) {
        line(img, cameraPoolCorners[i], cameraPoolCorners[(i+1)%cameraPoolCorners.size()], Scalar(0,0,255), 4);
        //cout<<cameraPoolCorners[i]<<"   ";
    }
    //for (int i=0; i<cameraPoolCorners.size(); i++) {
    //    cout<<cameraPoolCorners[i]<<"   ";
    //}
    //cout<<endl;
    
    return cameraPoolCorners;
}
