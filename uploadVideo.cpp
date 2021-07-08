//
//  uploadVideo.cpp
//  WaterPoloApp
//
//  Created by Chris Kreienkamp on 6/1/21.
//

#include "functions.hpp"
bool finishedInitializingPoolSides();
double getSlopeOf2Points(Point2f point1, Point2f point2);
double getYInterceptOf2Points(Point2f point1, Point2f point2);
Point2f getIntersectionFrom2SlopeInterceptLines(vector<double> side1, vector<double> side2);
Point2f centroidOfPoints(vector<Point2f> points);
void sortCornersClockwiseInPlaceWith0AtTopLeft(vector<Point2f> &corners);
void centerPointsOnScreenInPlace(vector<Point2f> &points);
vector<double> convertMatrixToVector(Mat matrix);

const Point2f UNKNOWN_POINT = Point2f(-INT_MIN, -INT_MIN);
vector<Point2f> eightPointsFormingPoolSides = {UNKNOWN_POINT, UNKNOWN_POINT, UNKNOWN_POINT, UNKNOWN_POINT, UNKNOWN_POINT, UNKNOWN_POINT, UNKNOWN_POINT, UNKNOWN_POINT};
Point2f* ptrToPointFormingPoolSide;
vector<string> possibleInputStates_POOL = {"move points", "move all", "reset points"};
string editMode = "move points";
bool mouseClicking = false;
Point2f mousePositionWhenClicked;
extern team location;
extern team away;
extern team home;

class poolIdentifier {
    vector<Point2f> points;
    vector<Point2f> corners;
    Point2f* pointer;
    
public:
    void addPoint(Point2f point) {
        points.push_back(point);
        pointer = &points.back();
        findCorners();
    }
    
    void movePoint(Point2f point) {
        *pointer = point;
        findCorners();
    }
    
    void moveAllPoints(double xTranslation, double yTranslation) {
        for (int i=0; i<points.size(); i++) {
            points[i] += Point2f(xTranslation, yTranslation);
        }
        findCorners();
    }
    
    void resetPoints() {
        points.clear();
    }
    
    void setPointerFromPositionInVector(int i) {
        pointer = &points[i];
    }
    
    vector<Point2f> getCorners() {
        if (corners.size() == 4) {
            sortCornersClockwiseInPlaceWith0AtTopLeft(corners);
        }
        return corners;
    }
    
    vector<Point2f> getPoints() {
        return points;
    }

private:
    void findCorners() {
        vector<Point2f> sortedPoints = sortPointsClockwiseAroundCentroid(points);
        if (points.size()>8) {
            vector<Point2f> mostSignificant8Points = reduceTo8MostSignificantPoints(sortedPoints);
            
            vector<double> slopes1, slopes2, yIntercepts1, yIntercepts2;
            for (int i=0; i<8; i++) {
                double slope = findSlope(mostSignificant8Points[i], mostSignificant8Points[(i+1)%8]);
                double yIntercept = findYIntercept(mostSignificant8Points[i], mostSignificant8Points[(i+1)%8]);
                if (i%2 == 0) {
                    slopes1.push_back(slope);
                    yIntercepts1.push_back(yIntercept);
                } else {
                    slopes2.push_back(slope);
                    yIntercepts2.push_back(yIntercept);
                }
            }
            
            double error1 = findMeanSquaredErrorOf4SlopeInterceptLines(slopes1, yIntercepts1);
            double error2 = findMeanSquaredErrorOf4SlopeInterceptLines(slopes2, yIntercepts2);
            
            if (error1 > error2) {
                mostSignificant8Points.push_back(mostSignificant8Points[0]);
                mostSignificant8Points.erase(mostSignificant8Points.begin());
            }
            corners = findCornersFrom8OrderedPoints(mostSignificant8Points);
        }
    }
    
    double findSlope(Point2f point1, Point2f point2) {
        double slope = static_cast<double>(point2.y - point1.y) / (point2.x - point1.x);
        return slope;
    }

    double findYIntercept(Point2f point1, Point2f point2) {
        double slope = findSlope(point1, point2);
        double yIntercept;
        if (slope == INFINITY) {
            yIntercept = point1.x;
        } else {
            yIntercept = point1.y - slope * point1.x;
        }
        return yIntercept;
    }
    
    Point2f getIntersectionFrom2SlopeInterceptLines(vector<double> side1, vector<double> side2) {
        Point2f intersection;
        double slope1 = side1[0];
        double intercept1 = side1[1];
        double slope2 = side2[0];
        double intercept2 = side2[1];
        if (slope1 == INFINITY) {
            intersection.x = intercept1;
            intersection.y = slope2*intersection.x + intercept2;
        } else if (slope2 == INFINITY) {
            intersection.x = intercept2;
            intersection.y = slope1*intersection.x + intercept1;
        } else {
            intersection.x = (intercept2 - intercept1) / (slope1 - slope2);
            intersection.y = slope1*intersection.x + intercept1;
        }
        return intersection;
    }
    
    vector<Point2f> sortPointsClockwiseAroundCentroid(vector<Point2f> pts) {
        Point2f poolCentroid = centroidOfPoints(pts);
        for (int i=0; i<pts.size(); i++) {
            for (int j=(int)(pts.size()-1); j>i; j--) {
                double centroidAngle1 = atan2(poolCentroid.y-pts[j-1].y, poolCentroid.x-pts[j-1].x);
                double centroidAngle2 = atan2(poolCentroid.y-pts[j].y, poolCentroid.x-pts[j].x);
                if (centroidAngle2 < centroidAngle1) {
                    Point2f minValue = pts[j];
                    pts[j] = pts[j-1];
                    pts[j-1] = minValue;
                }
            }
        }
        return pts;
    }
    
    Point2f centroidOfPoints(vector<Point2f> pts) {
        Point2f centroid = Point2f(0.0, 0.0);
        for (int i=0; i<pts.size(); i++) {
            centroid.x += pts[i].x;
            centroid.y += pts[i].y;
        }
        centroid.x = centroid.x / pts.size();
        centroid.y = centroid.y / pts.size();
        return centroid;
    }
    
    vector<Point2f> findCornersFrom8OrderedPoints(vector<Point2f> pts) {
        vector<Point2f> corners;
        vector<double> slopes, yIntercepts;
        for (int i=0; i<4; i++) {
            double slope = findSlope(pts[2*i], pts[2*i+1]);
            double yIntercept = findYIntercept(pts[2*i], pts[2*i+1]);
            slopes.push_back(slope);
            yIntercepts.push_back(yIntercept);
        }
        for (int i=0; i<4; i++) {
            Point2f corner = getIntersectionFrom2SlopeInterceptLines({slopes[i],yIntercepts[i]}, {slopes[(i+1)%4],yIntercepts[(i+1)%4]});
            corners.push_back(corner);
        }
        return corners;
    }
    
    vector<Point2f> reduceTo8MostSignificantPoints(vector<Point2f> pts) {
        vector<Point2f> mostSignificant8Points = pts;
        while (mostSignificant8Points.size() > 8) {
            double minimumDistance = INT_MAX;
            vector<Point2f>::iterator ptr = mostSignificant8Points.begin();
            
            for (int i=0; i<mostSignificant8Points.size(); i++) {
                int i1 = (i == 0) ? (int)mostSignificant8Points.size()-1 : i-1;
                Point2f point1 = mostSignificant8Points[i1];
                Point2f point2 = mostSignificant8Points[i];
                Point2f point3 = mostSignificant8Points[(i+1)%mostSignificant8Points.size()];
                double slope = findSlope(point1, point3);
                double yIntercept = findYIntercept(point1, point3);
                double dist = findPerpendicularDistanceFromPointToSlopeIntercept(point2, slope, yIntercept);
                if (dist < minimumDistance) {
                    minimumDistance = dist;
                    ptr = mostSignificant8Points.begin();
                    advance(ptr, i);
                }
            }
            mostSignificant8Points.erase(ptr);
        }
        return mostSignificant8Points;
    }
    
    double findMeanSquaredErrorOf4SlopeInterceptLines(vector<double> slopes, vector<double> intercepts) {
        double score = 0;
        for (int i=0; i<points.size(); i++) {
            double minDistance = INT_MAX;
            for (int j=0; j<4; j++) {
                double distance = findPerpendicularDistanceFromPointToSlopeIntercept(points[i], slopes[j], intercepts[j]);
                if (distance < minDistance) {
                    minDistance = distance;
                }
            }
            score += minDistance*minDistance;
        }
        score /= points.size();
        return score;
    }
    
    double findPerpendicularDistanceFromPointToSlopeIntercept(Point2f pt, double slope, double intercept) {
        double distance;
        if (abs(slope) == INFINITY) {
            distance = abs(pt.x-intercept);
        } else if (slope == 0) {
            distance = abs(pt.y-intercept);
        } else {
            double newSlope = -1.0/slope;
            double newIntercept = pt.x/slope + pt.y;
            Point2f intersection = getIntersectionFrom2SlopeInterceptLines({slope,intercept}, {newSlope,newIntercept});
            distance = sqrt(pow(pt.x-intersection.x,2) + pow(pt.y-intersection.y,2));
        }
        return distance;
    }
};






poolIdentifier pool;
void uploadVideo(string path) {
    
    // read in the first frame of the video
    VideoCapture video;
    video.open(path+"video.mp4");
    Mat firstFrame;
    video.read(firstFrame);
    namedWindow("First Frame of Video");
    setMouseCallback("First Frame of Video", getPoolCornersFromMouseInput);

    // make a points from the class individually moveable, moveable as a group, and removeable
    
    bool editingPoolCornerLocations = true;
    while (editingPoolCornerLocations) {
        drawFrameOfVideo(firstFrame);
        int keyboardPress = waitKey(1);
        if (finishedInitializingPoolSides()) {
            const char ESCAPE = 27;
            if (keyboardPress == ESCAPE) {editingPoolCornerLocations = false;}
            if (keyboardPress == '1') {editMode = "move points";}
            if (keyboardPress == '2') {editMode = "move all";}
            if (keyboardPress == '3') {editMode = "reset points";}
        }
    }
    
    vector<Point2f> poolCornersInCameraFrame = pool.getCorners();
    vector<Point2f> poolCornersInPoolFrame = {Point2f(0,0), Point2f(location.poolLength,0), Point2f(location.poolLength,location.poolWidth), Point2f(0,location.poolWidth)};
    Mat homographyFromCameraFrameToPoolFrame = findHomography(poolCornersInCameraFrame,poolCornersInPoolFrame);
    
    vector<double> homographyVector = convertMatrixToVector(homographyFromCameraFrameToPoolFrame);
    //ofstream saveHomographyToFile(path+"homography");
    //ostream_iterator<double> homographyIterator(saveHomographyToFile, "\n" );
    //copy(homographyVector.begin( ), homographyVector.end( ), homographyIterator);
}









vector<double> convertMatrixToVector(Mat matrix) {
    vector<double> vector;
    for (int i=0; i<matrix.rows; i++) {
        for (int j=0; j<matrix.cols; j++) {
            vector.push_back(matrix.at<double>(i,j));
        }
    }
    return vector;
}

void getPoolCornersFromMouseInput( int event, int x, int y, int flags, void* userdata) {
    if (event == EVENT_LBUTTONDOWN && !mouseClicking) {
        mouseClicking = true;
        
        if (!finishedInitializingPoolSides()) {
            pool.addPoint(Point2f(x,y));
        }
        
        else if (finishedInitializingPoolSides()) {
            if (editMode == "move points") {
                double distanceFromMouseToPointFormingPoolSide = INT_MAX;
                for (int i=0; i<pool.getPoints().size(); i++) {
                    if (distance(Point2f(x,y),pool.getPoints()[i]) < distanceFromMouseToPointFormingPoolSide) {
                        distanceFromMouseToPointFormingPoolSide = distance(Point2f(x,y),pool.getPoints()[i]);
                        pool.setPointerFromPositionInVector(i);
                    }
                }
                pool.movePoint(Point2f(x,y));
            } else if (editMode == "move all") {
                mousePositionWhenClicked = Point2f(x,y);
            } else if (editMode == "reset points") {
                pool.resetPoints();
                editMode = "move points";
            }
        }
    }
    
    else if (event == EVENT_MOUSEMOVE && mouseClicking) {
        
        if (!finishedInitializingPoolSides()) {
            pool.movePoint(Point2f(x,y));
        }
        
        else if (finishedInitializingPoolSides()) {
            if (editMode == "move points") {
                pool.movePoint(Point2f(x,y));
            } else if (editMode == "move all") {
                double xTranslation = double(x) - mousePositionWhenClicked.x;
                double yTranslation = double(y) - mousePositionWhenClicked.y;
                pool.moveAllPoints(xTranslation, yTranslation);
                mousePositionWhenClicked = Point2f(x,y);
            }
        }
    }
    
    else if (event == EVENT_LBUTTONUP) {
        mouseClicking = false;
    }
}

bool finishedInitializingPoolSides() {
    //if (eightPointsFormingPoolSides.back() == UNKNOWN_POINT) {
    if (pool.getPoints().size() < 9) {
        return false;
    } else {
        return true;
    }
}

double distance(Point2f a, Point2f b) {
    return double (sqrt(pow(a.x-b.x, 2) + pow(a.y-b.y, 2)));
}

vector<Point2f> getPoolCornersFromPoolSidePoints() {
    vector<vector<double>> sidesInSlopeInterceptForm;
    for (int i=0; i<4; i++) {
        Point2f sidePoint1 = eightPointsFormingPoolSides[i*2];
        Point2f sidePoint2 = eightPointsFormingPoolSides[i*2+1];
        double slope = getSlopeOf2Points(sidePoint1, sidePoint2);
        double yIntercept = getYInterceptOf2Points(sidePoint1, sidePoint2);
        sidesInSlopeInterceptForm.push_back( {slope,yIntercept} );
    }
    
    vector<Point2f> corners;
    for (int i=0; i<4; i++) {
        vector<double> side1slopeIntercept = sidesInSlopeInterceptForm[i];
        vector<double> side2slopeIntercept = sidesInSlopeInterceptForm[(i+1)%4];
        Point2f intersection = getIntersectionFrom2SlopeInterceptLines(side1slopeIntercept, side2slopeIntercept);
        corners.push_back(intersection);
    }
    
    sortCornersClockwiseInPlaceWith0AtTopLeft(corners);
    
    return corners;
}

double getSlopeOf2Points(Point2f point1, Point2f point2) {
    double slope = static_cast<double>(point2.y - point1.y) / (point2.x - point1.x);
    return slope;
}

double getYInterceptOf2Points(Point2f point1, Point2f point2) {
    double slope = getSlopeOf2Points(point1, point2);
    double yIntercept = point1.y - slope * point1.x;
    return yIntercept;
}

Point2f getIntersectionFrom2SlopeInterceptLines(vector<double> side1, vector<double> side2) {
    Point2f intersection;
    double slope1 = side1[0];
    double intercept1 = side1[1];
    double slope2 = side2[0];
    double intercept2 = side2[1];
    intersection.x = (intercept2 - intercept1) / (slope1 - slope2);
    intersection.y = slope1*intersection.x + intercept1;
    return intersection;
}

Point2f centroidOfPoints(vector<Point2f> points) {
    Point2f centroid = Point2f(0.0, 0.0);
    for (int i=0; i<points.size(); i++) {
        centroid.x += points[i].x;
        centroid.y += points[i].y;
    }
    centroid.x = centroid.x / points.size();
    centroid.y = centroid.y / points.size();
    return centroid;
}

void sortCornersClockwiseInPlaceWith0AtTopLeft(vector<Point2f> &corners) {
    Point2f poolCentroid = centroidOfPoints(corners);
    
    for (int i=0; i<4; i++) {
        for (int j=3; j>i; j--) {
            double centroidAngle1 = atan2(poolCentroid.y-corners[j-1].y, poolCentroid.x-corners[j-1].x);
            double centroidAngle2 = atan2(poolCentroid.y-corners[j].y, poolCentroid.x-corners[j].x);
            if (centroidAngle2 < centroidAngle1) {
                Point2f minValue = corners[j];
                corners[j] = corners[j-1];
                corners[j-1] = minValue;
            }
        }
    }
    
    Point2f holder1ForCameraPoolCorners = corners[0];
    Point2f holder2ForCameraPoolCorners = corners[1];
    corners[0] = corners[2];
    corners[1] = corners[3];
    corners[2] = holder1ForCameraPoolCorners;
    corners[3] = holder2ForCameraPoolCorners;
}

void drawFrameOfVideo(Mat frame) {
    Mat camMask = frame.clone();
    Mat camMaskPositive = Mat::zeros(frame.size(), frame.type());
    
    vector<Point2f> poolSidePoints = pool.getPoints();
    vector<Point2f> poolCorners = pool.getCorners();
    
    for (int i=0; i<poolSidePoints.size(); i++) {
        circle(camMaskPositive, poolSidePoints[i], 5, Scalar(100,100,100), FILLED);
    }
    
    if (finishedInitializingPoolSides()) {
        for (int i=0; i<poolCorners.size(); i++) {
            circle(camMaskPositive, poolCorners[i], 8, Scalar(100,100,100), FILLED);
            line(camMaskPositive, poolCorners[i], poolCorners[(i+1)%poolCorners.size()], Scalar(100,100,100), 3);
        }
    }
    
    // black bar on bottom
    rectangle(camMask, Point(0,frame.rows-50), Point(frame.cols,frame.rows), Scalar(0,0,0), FILLED);
    
    if (!finishedInitializingPoolSides()) {
        int numberSidesRemaining = 9-(int)pool.getPoints().size();
        //for (int i=0; i<eightPointsFormingPoolSides.size(); i++) {
        //    if (eightPointsFormingPoolSides[i] == UNKNOWN_POINT) {
        //        numberSidesRemaining++;
        //    }
        //}
        putText(camMask, "Instructions: ", Point(frame.cols*4.0/10.0-70, frame.rows-25), FONT_HERSHEY_PLAIN, 1.5, Scalar(255,255,255), 2);
        putText(camMask, "Choose "+to_string(numberSidesRemaining)+" remaining sides of pool", Point(frame.cols*5.0/10.0-70,frame.rows-25), FONT_HERSHEY_PLAIN, 1.5, Scalar(0,0,255), 2);
    } else {
        Scalar color;
        for (int i=0; i<possibleInputStates_POOL.size(); i++) {
            if (editMode == possibleInputStates_POOL[i]) {color = Scalar(0,0,255);} else {color = Scalar(255,255,255);}
            putText(camMask, to_string((i+1)%10)+": "+possibleInputStates_POOL[i], Point2f(frame.cols*(static_cast<double>(2*i+1))/(static_cast<double>(2*possibleInputStates_POOL.size()+1)), frame.rows-25), FONT_HERSHEY_PLAIN, 1.5, color, 2);
        }
    }

    add(camMask, camMaskPositive, camMask);
    imshow("First Frame of Video", camMask);
}
