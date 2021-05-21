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
#include <math.h>
#include "functions.hpp"

using namespace std;
using namespace cv;

bool COVID = true;
bool SAVEVIDEO = true;

Mat pool;
vector<vector<Point2f>> playerTail_vector;
vector<vector<double>> awayExposure_vector, homeExposure_vector;
deque<Scalar> playerTail_colors;
extern team location;
extern team away;
extern team home;
extern team ball;
double frames_per_second;
vector<double> awayExposureTotal = {0,0,0,0,0,0,0};
vector<double> homeExposureTotal = {0,0,0,0,0,0,0};
Mat camSmall;
Mat outputMat;
Mat stats;
int animatedScale = 20;                             // CAN CHANGE THIS

void presentation(string path)
{
    Mat cam;
    VideoCapture cap(path+"video.mp4");
    
    frames_per_second = cap.get(CAP_PROP_FPS);
    Mat waitbar = Mat(200,500, CV_8UC3);
    Mat outputMat;
    int frame_width, frame_height, frame_count = 0;
    unsigned long frameCounter = 0;
    frame_width = 1920*0.5+location.poolLength*animatedScale;
    frame_height = 1080*0.5;
    frame_count = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));
    Size frame_size(frame_width, frame_height);
    VideoWriter outputVideoWriter(path+"output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), frames_per_second, frame_size, true);
    
    // read in data
    vector<vector<Point2f>> awayPlayers_vector;
    vector<vector<Point2f>> homePlayers_vector;
    vector<Point2f> ball_vector;
    vector<vector<Point2f>> sides_vector;
    vector<vector<Point2f>> corners_vector;
    vector<vector<Point2f>> poolBoundary_vector;
    vector<vector<Point2f>> awayPlayers_vector_anim;
    vector<vector<Point2f>> homePlayers_vector_anim;
    vector<Point2f> ball_vector_anim;
    vector<vector<Point2f>> poolBoundary_vector_anim;
    vector<double> awayExposure, homeExposure;
    
    // read in data
    ifstream awayPlayers_ifile(path+"awayPlayers");
    ifstream homePlayers_ifile(path+"homePlayers");
    ifstream ball_ifile(path+"ball");
    ifstream sides_ifile(path+"sides");
    ifstream corners_ifile(path+"corners");
    ifstream poolBoundary_ifile(path+"poolBoundary");
    ifstream awayPlayers_anim_ifile(path+"awayPlayers_anim");
    ifstream homePlayers_anim_ifile(path+"homePlayers_anim");
    ifstream ball_anim_ifile(path+"ball_anim");
    ifstream poolBoundary_anim_ifile(path+"poolBoundary_anim");
    ifstream playerTail_ifile(path+"playerTail");
    ifstream awayExposure_ifile(path+"awayExposure");
    ifstream homeExposure_ifile(path+"homeExposure");
    ifstream exposureScore_ifile(path+"exposureScore");
    double var1, var2, var3, var4, var5, var6, var7, var8, var9, var10, var11, var12, var13, var14, var15, var16, var17, var18, var19, var20, var21, var22, var23, var24, var25, var26, var27, var28, var29, var30, var31, var32, var33, var34, var35, var36, var37, var38, var39, var40, var41, var42, var43, var44, var45, var46, var47, var48, var49, var50, var51, var52, var53, var54, var55, var56, var57, var58, var59, var60, var61, var62, var63, var64, var65, var66, var67, var68, var69, var70, var71, var72, var73, var74, var75, var76, var77, var78, var79, var80, var81, var82, var83, var84, var85, var86, var87, var88, var89, var90, var91, var92, var93, var94, var95, var96, var97, var98, var99, var100, var101, var102, var103, var104, var105, var106, var107, var108, var109, var110, var111, var112, var113, var114, var115, var116, var117, var118, var119, var120, var121, var122, var123, var124, var125, var126, var127, var128, var129, var130, var131, var132, var133, var134, var135, var136, var137, var138, var139, var140, var141, var142, var143, var144, var145, var146, var147, var148, var149, var150, var151, var152, var153, var154, var155, var156, var157, var158, var159, var160, var161, var162, var163, var164, var165, var166, var167, var168, var169, var170, var171, var172, var173, var174, var175, var176, var177, var178, var179, var180, var181, var182, var183, var184, var185, var186, var187, var188, var189, var190, var191, var192, var193, var194, var195, var196, var197, var198, var199, var200, var201, var202, var203, var204, var205, var206, var207, var208, var209, var210, var211, var212, var213, var214, var215, var216, var217, var218, var219, var220, var221, var222, var223, var224, var225, var226, var227, var228, var229, var230, var231, var232, var233, var234, var235, var236, var237, var238, var239, var240, var241, var242, var243, var244, var245, var246, var247, var248, var249, var250, var251, var252, var253, var254, var255, var256, var257, var258, var259, var260, var261, var262, var263, var264, var265, var266, var267, var268, var269, var270, var271, var272, var273, var274, var275, var276, var277, var278, var279, var280, var281, var282, var283, var284, var285, var286, var287, var288, var289, var290, var291, var292, var293, var294, var295, var296, var297, var298, var299, var300, var301, var302, var303, var304, var305, var306, var307, var308, var309, var310, var311, var312, var313, var314, var315, var316, var317, var318, var319, var320, var321, var322, var323, var324, var325, var326, var327, var328, var329, var330, var331, var332, var333, var334, var335, var336, var337, var338, var339, var340, var341, var342, var343, var344, var345, var346, var347, var348, var349, var350, var351, var352, var353, var354, var355, var356, var357, var358, var359, var360, var361, var362, var363, var364, var365, var366, var367, var368, var369, var370, var371, var372, var373, var374, var375, var376, var377, var378, var379, var380, var381, var382, var383, var384, var385, var386, var387, var388, var389, var390, var391, var392, var393, var394, var395, var396, var397, var398, var399, var400;
    char c;
    while (awayPlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_ifile >> c >> var1 >> c >> var2 >> c) {ball_vector.push_back(Point2f(var1,var2));}
    while (sides_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {sides_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (corners_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c) {corners_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8)});}
    while (poolBoundary_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (awayPlayers_anim_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {awayPlayers_vector_anim.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (homePlayers_anim_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c) {homePlayers_vector_anim.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14)});}
    while (ball_anim_ifile >> c >> var1 >> c >> var2 >> c) {ball_vector_anim.push_back(Point2f(var1,var2));}
    while (poolBoundary_anim_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c) {poolBoundary_vector_anim.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16)});}
    while (playerTail_ifile >> c >> var1 >> c >> var2 >> c >> var3 >> c >> var4 >> c >> var5 >> c >> var6 >> c >> var7 >> c >> var8 >> c >> var9 >> c >> var10 >> c >> var11 >> c >> var12 >> c >> var13 >> c >> var14 >> c >> var15 >> c >> var16 >> c >> var17 >> c >> var18 >> c >> var19 >> c >> var20 >> c >> var21 >> c >> var22 >> c >> var23 >> c >> var24 >> c >> var25 >> c >> var26 >> c >> var27 >> c >> var28 >> c >> var29 >> c >> var30 >> c >> var31 >> c >> var32 >> c >> var33 >> c >> var34 >> c >> var35 >> c >> var36 >> c >> var37 >> c >> var38 >> c >> var39 >> c >> var40 >> c >> var41 >> c >> var42 >> c >> var43 >> c >> var44 >> c >> var45 >> c >> var46 >> c >> var47 >> c >> var48 >> c >> var49 >> c >> var50 >> c >> var51 >> c >> var52 >> c >> var53 >> c >> var54 >> c >> var55 >> c >> var56 >> c >> var57 >> c >> var58 >> c >> var59 >> c >> var60 >> c >> var61 >> c >> var62 >> c >> var63 >> c >> var64 >> c >> var65 >> c >> var66 >> c >> var67 >> c >> var68 >> c >> var69 >> c >> var70 >> c >> var71 >> c >> var72 >> c >> var73 >> c >> var74 >> c >> var75 >> c >> var76 >> c >> var77 >> c >> var78 >> c >> var79 >> c >> var80 >> c >> var81 >> c >> var82 >> c >> var83 >> c >> var84 >> c >> var85 >> c >> var86 >> c >> var87 >> c >> var88 >> c >> var89 >> c >> var90 >> c >> var91 >> c >> var92 >> c >> var93 >> c >> var94 >> c >> var95 >> c >> var96 >> c >> var97 >> c >> var98 >> c >> var99 >> c >> var100 >> c >> var101 >> c >> var102 >> c >> var103 >> c >> var104 >> c >> var105 >> c >> var106 >> c >> var107 >> c >> var108 >> c >> var109 >> c >> var110 >> c >> var111 >> c >> var112 >> c >> var113 >> c >> var114 >> c >> var115 >> c >> var116 >> c >> var117 >> c >> var118 >> c >> var119 >> c >> var120 >> c >> var121 >> c >> var122 >> c >> var123 >> c >> var124 >> c >> var125 >> c >> var126 >> c >> var127 >> c >> var128 >> c >> var129 >> c >> var130 >> c >> var131 >> c >> var132 >> c >> var133 >> c >> var134 >> c >> var135 >> c >> var136 >> c >> var137 >> c >> var138 >> c >> var139 >> c >> var140 >> c >> var141 >> c >> var142 >> c >> var143 >> c >> var144 >> c >> var145 >> c >> var146 >> c >> var147 >> c >> var148 >> c >> var149 >> c >> var150 >> c >> var151 >> c >> var152 >> c >> var153 >> c >> var154 >> c >> var155 >> c >> var156 >> c >> var157 >> c >> var158 >> c >> var159 >> c >> var160 >> c >> var161 >> c >> var162 >> c >> var163 >> c >> var164 >> c >> var165 >> c >> var166 >> c >> var167 >> c >> var168 >> c >> var169 >> c >> var170 >> c >> var171 >> c >> var172 >> c >> var173 >> c >> var174 >> c >> var175 >> c >> var176 >> c >> var177 >> c >> var178 >> c >> var179 >> c >> var180 >> c >> var181 >> c >> var182 >> c >> var183 >> c >> var184 >> c >> var185 >> c >> var186 >> c >> var187 >> c >> var188 >> c >> var189 >> c >> var190 >> c >> var191 >> c >> var192 >> c >> var193 >> c >> var194 >> c >> var195 >> c >> var196 >> c >> var197 >> c >> var198 >> c >> var199 >> c >> var200 >> c >> var201 >> c >> var202 >> c >> var203 >> c >> var204 >> c >> var205 >> c >> var206 >> c >> var207 >> c >> var208 >> c >> var209 >> c >> var210 >> c >> var211 >> c >> var212 >> c >> var213 >> c >> var214 >> c >> var215 >> c >> var216 >> c >> var217 >> c >> var218 >> c >> var219 >> c >> var220 >> c >> var221 >> c >> var222 >> c >> var223 >> c >> var224 >> c >> var225 >> c >> var226 >> c >> var227 >> c >> var228 >> c >> var229 >> c >> var230 >> c >> var231 >> c >> var232 >> c >> var233 >> c >> var234 >> c >> var235 >> c >> var236 >> c >> var237 >> c >> var238 >> c >> var239 >> c >> var240 >> c >> var241 >> c >> var242 >> c >> var243 >> c >> var244 >> c >> var245 >> c >> var246 >> c >> var247 >> c >> var248 >> c >> var249 >> c >> var250 >> c >> var251 >> c >> var252 >> c >> var253 >> c >> var254 >> c >> var255 >> c >> var256 >> c >> var257 >> c >> var258 >> c >> var259 >> c >> var260 >> c >> var261 >> c >> var262 >> c >> var263 >> c >> var264 >> c >> var265 >> c >> var266 >> c >> var267 >> c >> var268 >> c >> var269 >> c >> var270 >> c >> var271 >> c >> var272 >> c >> var273 >> c >> var274 >> c >> var275 >> c >> var276 >> c >> var277 >> c >> var278 >> c >> var279 >> c >> var280 >> c >> var281 >> c >> var282 >> c >> var283 >> c >> var284 >> c >> var285 >> c >> var286 >> c >> var287 >> c >> var288 >> c >> var289 >> c >> var290 >> c >> var291 >> c >> var292 >> c >> var293 >> c >> var294 >> c >> var295 >> c >> var296 >> c >> var297 >> c >> var298 >> c >> var299 >> c >> var300 >> c >> var301 >> c >> var302 >> c >> var303 >> c >> var304 >> c >> var305 >> c >> var306 >> c >> var307 >> c >> var308 >> c >> var309 >> c >> var310 >> c >> var311 >> c >> var312 >> c >> var313 >> c >> var314 >> c >> var315 >> c >> var316 >> c >> var317 >> c >> var318 >> c >> var319 >> c >> var320 >> c >> var321 >> c >> var322 >> c >> var323 >> c >> var324 >> c >> var325 >> c >> var326 >> c >> var327 >> c >> var328 >> c >> var329 >> c >> var330 >> c >> var331 >> c >> var332 >> c >> var333 >> c >> var334 >> c >> var335 >> c >> var336 >> c >> var337 >> c >> var338 >> c >> var339 >> c >> var340 >> c >> var341 >> c >> var342 >> c >> var343 >> c >> var344 >> c >> var345 >> c >> var346 >> c >> var347 >> c >> var348 >> c >> var349 >> c >> var350 >> c >> var351 >> c >> var352 >> c >> var353 >> c >> var354 >> c >> var355 >> c >> var356 >> c >> var357 >> c >> var358 >> c >> var359 >> c >> var360 >> c >> var361 >> c >> var362 >> c >> var363 >> c >> var364 >> c >> var365 >> c >> var366 >> c >> var367 >> c >> var368 >> c >> var369 >> c >> var370 >> c >> var371 >> c >> var372 >> c >> var373 >> c >> var374 >> c >> var375 >> c >> var376 >> c >> var377 >> c >> var378 >> c >> var379 >> c >> var380 >> c >> var381 >> c >> var382 >> c >> var383 >> c >> var384 >> c >> var385 >> c >> var386 >> c >> var387 >> c >> var388 >> c >> var389 >> c >> var390 >> c >> var391 >> c >> var392 >> c >> var393 >> c >> var394 >> c >> var395 >> c >> var396 >> c >> var397 >> c >> var398 >> c >> var399 >> c >> var400 >> c) {playerTail_vector.push_back({Point2f(var1,var2), Point2f(var3,var4), Point2f(var5,var6), Point2f(var7,var8), Point2f(var9,var10), Point2f(var11,var12), Point2f(var13,var14), Point2f(var15,var16), Point2f(var17,var18), Point2f(var19,var20), Point2f(var21,var22), Point2f(var23,var24), Point2f(var25,var26), Point2f(var27,var28), Point2f(var29,var30), Point2f(var31,var32), Point2f(var33,var34), Point2f(var35,var36), Point2f(var37,var38), Point2f(var39,var40), Point2f(var41,var42), Point2f(var43,var44), Point2f(var45,var46), Point2f(var47,var48), Point2f(var49,var50), Point2f(var51,var52), Point2f(var53,var54), Point2f(var55,var56), Point2f(var57,var58), Point2f(var59,var60), Point2f(var61,var62), Point2f(var63,var64), Point2f(var65,var66), Point2f(var67,var68), Point2f(var69,var70), Point2f(var71,var72), Point2f(var73,var74), Point2f(var75,var76), Point2f(var77,var78), Point2f(var79,var80), Point2f(var81,var82), Point2f(var83,var84), Point2f(var85,var86), Point2f(var87,var88), Point2f(var89,var90), Point2f(var91,var92), Point2f(var93,var94), Point2f(var95,var96), Point2f(var97,var98), Point2f(var99,var100), Point2f(var101,var102), Point2f(var103,var104), Point2f(var105,var106), Point2f(var107,var108), Point2f(var109,var110), Point2f(var111,var112), Point2f(var113,var114), Point2f(var115,var116), Point2f(var117,var118), Point2f(var119,var120), Point2f(var121,var122), Point2f(var123,var124), Point2f(var125,var126), Point2f(var127,var128), Point2f(var129,var130), Point2f(var131,var132), Point2f(var133,var134), Point2f(var135,var136), Point2f(var137,var138), Point2f(var139,var140), Point2f(var141,var142), Point2f(var143,var144), Point2f(var145,var146), Point2f(var147,var148), Point2f(var149,var150), Point2f(var151,var152), Point2f(var153,var154), Point2f(var155,var156), Point2f(var157,var158), Point2f(var159,var160), Point2f(var161,var162), Point2f(var163,var164), Point2f(var165,var166), Point2f(var167,var168), Point2f(var169,var170), Point2f(var171,var172), Point2f(var173,var174), Point2f(var175,var176), Point2f(var177,var178), Point2f(var179,var180), Point2f(var181,var182), Point2f(var183,var184), Point2f(var185,var186), Point2f(var187,var188), Point2f(var189,var190), Point2f(var191,var192), Point2f(var193,var194), Point2f(var195,var196), Point2f(var197,var198), Point2f(var199,var200), Point2f(var201,var202), Point2f(var203,var204), Point2f(var205,var206), Point2f(var207,var208), Point2f(var209,var210), Point2f(var211,var212), Point2f(var213,var214), Point2f(var215,var216), Point2f(var217,var218), Point2f(var219,var220), Point2f(var221,var222), Point2f(var223,var224), Point2f(var225,var226), Point2f(var227,var228), Point2f(var229,var230), Point2f(var231,var232), Point2f(var233,var234), Point2f(var235,var236), Point2f(var237,var238), Point2f(var239,var240), Point2f(var241,var242), Point2f(var243,var244), Point2f(var245,var246), Point2f(var247,var248), Point2f(var249,var250), Point2f(var251,var252), Point2f(var253,var254), Point2f(var255,var256), Point2f(var257,var258), Point2f(var259,var260), Point2f(var261,var262), Point2f(var263,var264), Point2f(var265,var266), Point2f(var267,var268), Point2f(var269,var270), Point2f(var271,var272), Point2f(var273,var274), Point2f(var275,var276), Point2f(var277,var278), Point2f(var279,var280), Point2f(var281,var282), Point2f(var283,var284), Point2f(var285,var286), Point2f(var287,var288), Point2f(var289,var290), Point2f(var291,var292), Point2f(var293,var294), Point2f(var295,var296), Point2f(var297,var298), Point2f(var299,var300), Point2f(var301,var302), Point2f(var303,var304), Point2f(var305,var306), Point2f(var307,var308), Point2f(var309,var310), Point2f(var311,var312), Point2f(var313,var314), Point2f(var315,var316), Point2f(var317,var318), Point2f(var319,var320), Point2f(var321,var322), Point2f(var323,var324), Point2f(var325,var326), Point2f(var327,var328), Point2f(var329,var330), Point2f(var331,var332), Point2f(var333,var334), Point2f(var335,var336), Point2f(var337,var338), Point2f(var339,var340), Point2f(var341,var342), Point2f(var343,var344), Point2f(var345,var346), Point2f(var347,var348), Point2f(var349,var350), Point2f(var351,var352), Point2f(var353,var354), Point2f(var355,var356), Point2f(var357,var358), Point2f(var359,var360), Point2f(var361,var362), Point2f(var363,var364), Point2f(var365,var366), Point2f(var367,var368), Point2f(var369,var370), Point2f(var371,var372), Point2f(var373,var374), Point2f(var375,var376), Point2f(var377,var378), Point2f(var379,var380), Point2f(var381,var382), Point2f(var383,var384), Point2f(var385,var386), Point2f(var387,var388), Point2f(var389,var390), Point2f(var391,var392), Point2f(var393,var394), Point2f(var395,var396), Point2f(var397,var398), Point2f(var399,var400)});}//, Point2f(var401,var402)});}
    while (awayExposure_ifile >> var1) {awayExposure.push_back(var1);}
    while (homeExposure_ifile >> var1) {homeExposure.push_back(var1);}
    for (int i=0; i<awayExposure.size(); i+=7) {
        awayExposure_vector.push_back({awayExposure[i], awayExposure[i+1], awayExposure[i+2], awayExposure[i+3], awayExposure[i+4], awayExposure[i+5], awayExposure[i+6]});
        homeExposure_vector.push_back({homeExposure[i], homeExposure[i+1], homeExposure[i+2], homeExposure[i+3], homeExposure[i+4], homeExposure[i+5], homeExposure[i+6]});
    }
    
    // produce a plot of the exposure score data
    vector<double> exposureScore;
    vector<vector<double>> exposureScore_vector;
    //vector<Scalar> plotColors;
    while (exposureScore_ifile >> var1) {exposureScore.push_back(var1);}
    Mat exposurePlot = Mat(800,1500, CV_8UC3, Scalar(255,255,255));
    line(exposurePlot, Point(75,700), Point(1425,700), Scalar(0,0,0), 3);
    putText(exposurePlot, "0", Point(60,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "5", Point(210,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "10", Point(360,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "15", Point(510,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "20", Point(660,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "25", Point(810,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "30", Point(960,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "35", Point(1110,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "40", Point(1260,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "45", Point(1410,750), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2);
    for (int i=0; i<exposureScore.size(); i+=14) {
        exposureScore_vector.push_back({exposureScore[i], exposureScore[i+1], exposureScore[i+2], exposureScore[i+3], exposureScore[i+4], exposureScore[i+5], exposureScore[i+6], exposureScore[i+7], exposureScore[i+8], exposureScore[i+9], exposureScore[i+10], exposureScore[i+11], exposureScore[i+12], exposureScore[i+13]});
        //int scalarNumber = 255*3*i/exposureScore.size();
        //if (scalarNumber>255*2) {plotColors.push_back(Scalar(100,0,scalarNumber%255));}
        //else if (scalarNumber>255) {plotColors.push_back(Scalar(0,scalarNumber%255,100));}
        //else {plotColors.push_back(Scalar(scalarNumber%255,100,0));}
    }
    for (int i=0; i<exposureScore_vector.size(); i++) {
        if (i == 0) {putText(exposurePlot, "G", Point(1450,650-40*i), FONT_HERSHEY_SIMPLEX, 1, away.primaryColor,2);}
        else if (i == 7) {putText(exposurePlot, "G", Point(1450,650-40*i), FONT_HERSHEY_SIMPLEX, 1, home.primaryColor,2);}
        else if (i<7) {putText(exposurePlot, to_string(i), Point(1450,650-40*i), FONT_HERSHEY_SIMPLEX, 1, away.primaryColor,2);}
        else {putText(exposurePlot, to_string(i), Point(1450,650-40*i), FONT_HERSHEY_SIMPLEX, 1, home.primaryColor,2);}
        for (int j=0; j<exposureScore_vector[0].size(); j++) {
            int positionx = 75+exposureScore_vector[i][j]/(awayPlayers_vector.size()/frames_per_second)*(1425-75);
            if (j==0) {circle(exposurePlot, Point(positionx, 650-40*i), 5, Scalar(0,255,0), FILLED);}
            else if (j==7) {circle(exposurePlot, Point(positionx, 650-40*i), 5, Scalar(0,0,0), FILLED);}
            else if (j<7) {circle(exposurePlot, Point(positionx, 650-40*i), 5, away.primaryColor, FILLED);}
            else {circle(exposurePlot, Point(positionx, 650-40*i), 5, home.primaryColor, FILLED);}
        }
    }
    imshow("exposure plot", exposurePlot);
    //waitKey(0);
    
    if (COVID) {
        // create colors for the (playerTail)
        // start out at 6.64 seconds ago (when 0.1 of the original exposure remains) so each successive color needs to get darker
        for (int i=0; i<playerTail_vector[0].size(); i++) {
            double timeBeforeCurrentFrame = (playerTail_vector[0].size()-1-i)/frames_per_second;
            double exposureScore = exp(log(0.5)/2.0*timeBeforeCurrentFrame);
            double colorNumbers2Add = (1-exposureScore)/0.9*(255*3);
            if (colorNumbers2Add > 510) {playerTail_colors.push_back(Scalar(colorNumbers2Add-510,255,255));}
            else if (colorNumbers2Add > 255) {playerTail_colors.push_back(Scalar(0,colorNumbers2Add-255,255));}
            else {playerTail_colors.push_back(Scalar(0,0,colorNumbers2Add));}
        }
    }
    
    // show the video
    int progress = -1;
    if (SAVEVIDEO) {cout<<"PRESENTATION: [0........10........20........30........40........50........60........70........80........90.......100]"<<endl<<"              [";}
    unsigned int iterator = 0;
    while (true) {
        cap.read(cam);
        if (cam.empty()) {break;}
        
        drawCamera_PRESENTATION(cam, iterator, awayPlayers_vector, homePlayers_vector, ball_vector, corners_vector);
        drawAnimated_PRESENTATION(iterator, awayPlayers_vector_anim, homePlayers_vector_anim, ball_vector_anim, corners_vector, poolBoundary_vector_anim);
        if (COVID) {drawStatistics_PRESENTATION(iterator);}
        else {stats.create(camSmall.rows-pool.rows, pool.cols, CV_8UC3); stats.setTo(Scalar(0,0,0));}
        
        Mat pool_stats;
        vconcat(pool, stats, pool_stats);
        hconcat(camSmall, pool_stats, outputMat);
        iterator++;
        
        if (SAVEVIDEO) {
            outputVideoWriter.write(outputMat);    //write the video frame to the file
            
            // waitbar for saving a video
            frameCounter++;
            /*waitbar.setTo(cv::Scalar(175,175,175));
            rectangle(waitbar,Point(50,80),Point(400,120),Scalar(0,0,0),1); // black outline
            rectangle(waitbar,Point(51,81),Point(399,119),Scalar(255,255,255),FILLED); // white box fill*/
            double percentFinished = static_cast<double>(frameCounter)/frame_count;
            /*rectangle(waitbar, Point(51,81), Point(51+(399.0-51.0)*percentFinished,119), Scalar(0,0,255),FILLED); // red box fill
            putText(waitbar, to_string(static_cast<int>(100*percentFinished))+"%", Point(420,110), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0),1.5);
            imshow("waitbar",waitbar);*/
            cout<<".";
            if ((int)percentFinished > progress) {progress = percentFinished; cout<<progress%10;} // screen output
        } else {imshow("output", outputMat);}
        
        int key = waitKey(1);
        if (key==27) {break;}
    }
    if (SAVEVIDEO) {
        outputVideoWriter.release(); //flush and close the video file
        cout<<"0]"<<endl<<endl;
    }
}

void drawCamera_PRESENTATION(Mat cam, int iterator, vector<vector<Point2f>> awayPlayers_vector, vector<vector<Point2f>> homePlayers_vector, vector<Point2f> ball_vector, vector<vector<Point2f>> corners_vector) {
    
    Mat camMaskPositive = Mat::zeros(cam.size(), cam.type());
    
    // circle on each player and ball
    for (int i=0; i<awayPlayers_vector[0].size(); i++) {
        circle(cam, awayPlayers_vector[iterator][i], 8, away.primaryColor, FILLED);
        circle(cam, awayPlayers_vector[iterator][i], 9, away.secondaryColor, 2);
        circle(cam, homePlayers_vector[iterator][i], 8, home.primaryColor, FILLED);
        circle(cam, homePlayers_vector[iterator][i], 9, home.secondaryColor, 2);
    }
    circle(cam, ball_vector[iterator], 7, ball.primaryColor, FILLED);
    circle(cam, ball_vector[iterator], 8, ball.secondaryColor, 2);
    
    // lines for boundaries
    for (int i=0; i<corners_vector[0].size(); i++) {
        line(camMaskPositive, corners_vector[iterator][i], corners_vector[iterator][(i+1)%corners_vector[0].size()], Scalar(100,100,100), 3);
    }
    
    // imshow camera
    add(cam, camMaskPositive, cam);
    resize(cam, camSmall, Size(), 0.5, 0.5);
    //imshow("Camera", cam);
}

void drawAnimated_PRESENTATION(int iterator, vector<vector<Point2f>> awayPlayers_vector_anim, vector<vector<Point2f>> homePlayers_vector_anim, vector<Point2f> ball_vector_anim, vector<vector<Point2f>> corners_vector_anim, vector<vector<Point2f>> poolBoundary_vector_anim) {
    
    // pool colors
    Scalar poolBlue = Scalar(255,255,204);
    Scalar poolYellow = Scalar(0,255,255);
    Scalar poolRed = Scalar(0,0,255);
    Scalar poolWhite = Scalar(255,255,255);
    Vec3b darkerTint = {50,50,50};
    
    // take pool measurements
    float goalWidth = 3;
    int windowLength = location.poolLength*animatedScale;
    int windowHeight = location.poolWidth*animatedScale;
    
    /*// find the homography that converts the camera plane to the animated pool plane
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
    }*/
    
    // draw the animated pool
    pool.create(windowHeight, windowLength, CV_8UC3);
    pool.setTo(poolBlue);                      // create a blank pool canvas
    line(pool, Point(0,windowHeight/2-windowHeight*goalWidth/location.poolWidth/2), Point(0,windowHeight/2+windowHeight*goalWidth/location.poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength,windowHeight/2-windowHeight*goalWidth/location.poolWidth/2), Point(windowLength,windowHeight/2+windowHeight*goalWidth/location.poolWidth/2), poolWhite,5);      // draw the goal
    line(pool, Point(windowLength*2/location.poolLength,0), Point(windowLength*2/location.poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength-windowLength*2/location.poolLength,0), Point(windowLength-windowLength*2/location.poolLength,windowHeight), poolRed,3);      // 2 meters from goal line
    line(pool, Point(windowLength*6/location.poolLength,0), Point(windowLength*6/location.poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength-windowLength*6/location.poolLength,0), Point(windowLength-windowLength*6/location.poolLength,windowHeight), poolYellow,3);      // 6 meters from goal line
    line(pool, Point(windowLength/2,0), Point(windowLength/2,windowHeight), poolWhite,3);      // half pool

    // darken the area of the pool outside the camera field of view
    if (poolBoundary_vector_anim[iterator].size()) {
        vector<Point> poolBoundaryAnimatedWithoutUnknowns;
        for (int i=0; i<poolBoundary_vector_anim[iterator].size(); i++) {
            if (poolBoundary_vector_anim[iterator][i] != Point2f(-10,-10)) {poolBoundaryAnimatedWithoutUnknowns.push_back(Point(poolBoundary_vector_anim[iterator][i].x*animatedScale, poolBoundary_vector_anim[iterator][i].y*animatedScale));}
        }
        Mat poolMaskAnimated1 = Mat(pool.rows, pool.cols, pool.type(), Scalar(50,50,50));
        Mat poolMaskAnimated2 = Mat::zeros(pool.rows, pool.cols, pool.type());
        fillConvexPoly(poolMaskAnimated2, poolBoundaryAnimatedWithoutUnknowns, Scalar(50,50,50));
        subtract(poolMaskAnimated1, poolMaskAnimated2, poolMaskAnimated1);
        subtract(pool, poolMaskAnimated1, pool);
        for (int i=0; i<poolBoundaryAnimatedWithoutUnknowns.size(); i++) {
            line(pool, Point2f(poolBoundaryAnimatedWithoutUnknowns[i].x,poolBoundaryAnimatedWithoutUnknowns[i].y), Point2f(poolBoundaryAnimatedWithoutUnknowns[(i+1)%poolBoundaryAnimatedWithoutUnknowns.size()].x,poolBoundaryAnimatedWithoutUnknowns[(i+1)%poolBoundaryAnimatedWithoutUnknowns.size()].y), Scalar(255,0,0), 2.5);
        }
    }
    
    if (COVID) {
        // show the player tail
        for (int i=0; i<playerTail_vector[0].size(); i++) {
            if (playerTail_vector[iterator][i] == Point2f(-10,-10)) {continue;}
            else {circle(pool, Point2f(playerTail_vector[iterator][i].x*animatedScale,playerTail_vector[iterator][i].y*animatedScale), animatedScale*1.5, playerTail_colors[i], FILLED);}
        }
    }
    
    // circle on each away player
    for (int i=0; i<awayPlayers_vector_anim[iterator].size(); i++) {
        if (awayPlayers_vector_anim[iterator][i] != Point2f(-10,-10)) {
            circle(pool, awayPlayers_vector_anim[iterator][i]*animatedScale, 6, away.primaryColor, FILLED);
            circle(pool, awayPlayers_vector_anim[iterator][i]*animatedScale, 7, away.secondaryColor, 2);
        }
    }
    if (COVID) {
        // show a smaller circle on players that are in the danger zone
        Scalar Xcolor;
        for (int i=0; i<awayExposure_vector[0].size(); i++) {
            if (awayExposure_vector[iterator][i] != 0) {
                double colorNumbers2Add = (1-awayExposure_vector[iterator][i])/0.9*(255*3);
                if (colorNumbers2Add > 510) {Xcolor = Scalar(colorNumbers2Add-510,255,255);}
                else if (colorNumbers2Add > 255) {Xcolor = Scalar(0,colorNumbers2Add-255,255);}
                else {Xcolor = Scalar(0,0,colorNumbers2Add);}
                circle(pool, awayPlayers_vector_anim[iterator][i]*animatedScale, 5, Xcolor, FILLED);
            }
        }
    }
    
    // circle on each home player
    for (int i=0; i<homePlayers_vector_anim[iterator].size(); i++) {
        if (homePlayers_vector_anim[iterator][i] != Point2f(-10,-10)) {
            circle(pool, homePlayers_vector_anim[iterator][i]*animatedScale, 6, home.primaryColor, FILLED);
            circle(pool, homePlayers_vector_anim[iterator][i]*animatedScale, 7, home.secondaryColor, 2);
        }
    }
    if (COVID) {
        // show a smaller circle on players that are in the danger zone
        Scalar Xcolor;
        for (int i=0; i<awayExposure_vector[0].size(); i++) {
            if (homeExposure_vector[iterator][i] != 0) {
                double colorNumbers2Add = (1-homeExposure_vector[iterator][i])/0.9*(255*3);
                if (colorNumbers2Add > 510) {Xcolor = Scalar(colorNumbers2Add-510,255,255);}
                else if (colorNumbers2Add > 255) {Xcolor = Scalar(0,colorNumbers2Add-255,255);}
                else {Xcolor = Scalar(0,0,colorNumbers2Add);}
                circle(pool, homePlayers_vector_anim[iterator][i]*animatedScale, 5, Xcolor, FILLED);
            }
        }
    }
    
    // circle on the ball
    if (ball_vector_anim[iterator] != Point2f(-10,-10)) {
        circle(pool, ball_vector_anim[iterator]*animatedScale, 5, ball.primaryColor, FILLED);
        circle(pool, ball_vector_anim[iterator]*animatedScale, 6, ball.secondaryColor, 2);
    }
}

void drawStatistics_PRESENTATION(int itFrame) {
    
    Mat exposurePlot;
    
    // draw the plot without numbers
    exposurePlot.create(camSmall.rows-pool.rows, pool.cols, CV_8UC3);
    exposurePlot.setTo(Scalar(255,255,255));                      // create a blank canvas
    putText(exposurePlot, "AWAY", Point2f(10,exposurePlot.rows*5.0/12.0), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 2);
    putText(exposurePlot, "HOME", Point2f(10,exposurePlot.rows*9.0/12.0), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 2);
    vector<string> playerNumberString = {"P1", "P2", "P3", "P4", "P5", "P6", "P7"};
    for (int i=0; i<awayExposure_vector[0].size(); i++) {
        putText(exposurePlot, playerNumberString[i], Point2f(pool.cols*(i+1)/8.0+10,exposurePlot.rows*2.0/12.0), FONT_HERSHEY_PLAIN, 1, Scalar(0,0,0), 1);
    }
    
    // color the boxes
    Scalar boxColor;
    for (int i=0; i<awayExposure_vector[0].size(); i++) {
        if (awayExposure_vector[itFrame][i] != 0) {
            double colorNumbers2Add = (1-awayExposure_vector[itFrame][i])/0.9*(255*3);
            if (colorNumbers2Add > 510) {boxColor = Scalar(colorNumbers2Add-510,255,255);}
            else if (colorNumbers2Add > 255) {boxColor = Scalar(0,colorNumbers2Add-255,255);}
            else {boxColor = Scalar(0,0,colorNumbers2Add);}
            rectangle(exposurePlot, Point2f(pool.cols*(i+1)/8.0,exposurePlot.rows*3.0/12.0), Point2f(pool.cols*(i+1)/8.0+60,exposurePlot.rows*6.0/12.0), boxColor, FILLED);
            rectangle(exposurePlot, Point2f(pool.cols*(i+1)/8.0,exposurePlot.rows*3.0/12.0), Point2f(pool.cols*(i+1)/8.0+60,exposurePlot.rows*6.0/12.0), Scalar(0,0,0), 1);
        }
        if (homeExposure_vector[itFrame][i] != 0) {
            double colorNumbers2Add = (1-homeExposure_vector[itFrame][i])/0.9*(255*3);
            if (colorNumbers2Add > 510) {boxColor = Scalar(colorNumbers2Add-510,255,255);}
            else if (colorNumbers2Add > 255) {boxColor = Scalar(0,colorNumbers2Add-255,255);}
            else {boxColor = Scalar(0,0,colorNumbers2Add);}
            rectangle(exposurePlot, Point2f(pool.cols*(i+1)/8.0,exposurePlot.rows*7.0/12.0), Point2f(pool.cols*(i+1)/8.0+60,exposurePlot.rows*10.0/12.0), boxColor, FILLED);
            rectangle(exposurePlot, Point2f(pool.cols*(i+1)/8.0,exposurePlot.rows*7.0/12.0), Point2f(pool.cols*(i+1)/8.0+60,exposurePlot.rows*10.0/12.0), Scalar(0,0,0), 1);
        }
    }
    
    // add the exposure score for each player
    for (int i=0; i<awayExposure_vector[0].size(); i++) {
        awayExposureTotal[i] += awayExposure_vector[itFrame][i]/frames_per_second;
        homeExposureTotal[i] += homeExposure_vector[itFrame][i]/frames_per_second;
        std::stringstream awayStream, homeStream;
        awayStream << std::fixed << std::setprecision(1) << awayExposureTotal[i];
        homeStream << std::fixed << std::setprecision(1) << homeExposureTotal[i];
        std::string awayS = awayStream.str();
        std::string homeS = homeStream.str();
        Scalar awayNumberColor = (awayExposure_vector[itFrame][i] >= 0.8) ? Scalar(255,255,255) : Scalar(0,0,0);
        Scalar homeNumberColor = (homeExposure_vector[itFrame][i] >= 0.8) ? Scalar(255,255,255) : Scalar(0,0,0);
        putText(exposurePlot, awayS, Point2f(pool.cols*(i+1)/8.0+10,exposurePlot.rows*5.0/12.0), FONT_HERSHEY_PLAIN, 1, awayNumberColor, 1);
        putText(exposurePlot, homeS, Point2f(pool.cols*(i+1)/8.0+10,exposurePlot.rows*9.0/12.0), FONT_HERSHEY_PLAIN, 1, homeNumberColor, 1);
    }
    
    // imshow
    //imshow("statistics", exposurePlot);
    stats = exposurePlot;
    
}
