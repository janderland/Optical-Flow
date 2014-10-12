//
//  main.cpp
//  OpticalFlow
//
//  Created by Jon Anderson on 10/8/14.
//  Copyright (c) 2014 Jon Anderson. All rights reserved.
//

#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

// General settings
#define Z_DEPTH 133.4f // cm
#define FOCAL_LENGTH .00342f // cm
#define LENGTH_PER_PIXEL 0.0000049f // cm
#define ITERATIONS_PER_SEED 5

// Display settings
// Defining "OPTFLOW_DISPLAY" (#define OPTFLOW_DISPLAY) enables graphical output for this application. This normally shouldn't be done in the source code but rather done in the IDE or Makefile as to not interfere with different build methods.
#define OVERLAY_CIRCLE_RADIUS 5
#define OVERLAY_COLOR_R 255
#define OVERLAY_COLOR_B 0
#define OVERLAY_COLOR_G 0

// Shi-Tomasi settings. Used when finding the seed corners
#define SHITOMASI_MAX_CORNERS 100
#define SHITOMASI_QUALITY_LEVEL 0.3f
#define SHITOMASI_MIN_DISTANCE 7
#define SHITOMASI_BLOCK_SIZE 7

// Lucas-Kanad Optical Flow settings. Used to track the seed corners until next seed.
#define LUCASKANAD_WINDOW_SIZE_X 15
#define LUCASKANAD_WINDOW_SIZE_Y 15
#define LUCASKANAD_MAX_LEVEL 2

const Scalar color(OVERLAY_COLOR_B, OVERLAY_COLOR_G, OVERLAY_COLOR_R);
const double multiplier = (Z_DEPTH / FOCAL_LENGTH) * LENGTH_PER_PIXEL;

int main(int argc, const char * argv[]) {
    // The change in position of the camera
    float dx, dy;
    dx = dy = 0;
    
    // The change in position for a single iteration
    float ddx, ddy;
    
    Mat cur_frame;
    Mat cur_fgray;
    Mat old_fgray;
    
#ifdef OPTFLOW_DISPLAY
    // Optical flow overlay
    Mat overlay;
#endif
    
    // Corner points being tracked
    vector<Point2f> old_p, cur_p, found_p;
    
    vector<unsigned char> status;
    vector<float> err;
    
    // Lucas Kanad Optical Flow parameters
    Size winSize(LUCASKANAD_WINDOW_SIZE_X, LUCASKANAD_WINDOW_SIZE_Y);
    TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 0.03);
    
    VideoCapture cap(-1);
    if (!cap.isOpened()) {
        cout << "Failed to open camera.\n";
        return 1;
    }
    
    while (true) {
        // Get a new frame
        cap >> cur_frame;
        cvtColor(cur_frame, old_fgray, CV_BGR2GRAY);
        
        // Get new corners from this frame
        goodFeaturesToTrack(old_fgray,
                            old_p,
                            SHITOMASI_MAX_CORNERS,
                            SHITOMASI_QUALITY_LEVEL,
                            SHITOMASI_MIN_DISTANCE,
                            noArray(),
                            SHITOMASI_BLOCK_SIZE);
        
        // If we didn't get get feature to track, try again
        if (old_p.size() == 0) {
            continue;
        }
       
#ifdef OPTFLOW_DISPLAY
        // Reset the mask
        overlay = Mat::zeros(cur_frame.size(), cur_frame.type());
#endif

        // Track these corners over the next few frames
        for (int i=0; i<ITERATIONS_PER_SEED; i++) {
            
            // Get a new frame
            cap >> cur_frame;
            cvtColor(cur_frame, cur_fgray, CV_BGR2GRAY);

            // Track corners over the current frame
            calcOpticalFlowPyrLK(old_fgray,
                                 cur_fgray,
                                 old_p,
                                 cur_p,
                                 status,
                                 err,
                                 winSize,
                                 LUCASKANAD_MAX_LEVEL,
                                 criteria);
            
            found_p.clear();
            ddx = ddy = 0;
            
            // Iterate over the corners
            for (int k=0; k<status.size(); k++) {
                if (status[k] > 0) {
                    // If the status value is 1 then this corner was found so we add it to the found list
                    found_p.push_back(cur_p[k]);
                    
                    // Add to the running sum of ddx/ddy
                    ddx += cur_p[k].x - old_p[k].x;
                    ddy += cur_p[k].y - old_p[k].y;
                    
#ifdef OPTFLOW_DISPLAY
                    // Draw the optical flow lines onto the mask
                    circle(cur_frame, cur_p[k], OVERLAY_CIRCLE_RADIUS, color, -1);
                    line(overlay, old_p[k], cur_p[k], color);
#endif
                }
            }
            
            // If no corners were tracked this iteration, reseed
            if (found_p.size() == 0) {
                break;
            }
            
            // Calculate the change in position for this iteration
            dx += (ddx / found_p.size()) * multiplier;
            dy += (ddy / found_p.size()) * multiplier;
            
            // Use all the tracked points in the next iteration (points that couldn't be tracked are thrown out)
            old_p.clear();
            old_p = found_p;
            
            cout << "dx: " << dx << " dy: " << dy << endl;
            
#ifdef OPTFLOW_DISPLAY
            // Display frame
            add(overlay, cur_frame, cur_frame);
            imshow("frame", cur_frame);
            if(waitKey(30) >= 0) break;
#endif
        }
    }
    
    return 0;
}
