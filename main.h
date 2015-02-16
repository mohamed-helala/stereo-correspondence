//// main.cpp: implementation of the main method. ///////////////////////
//  -getparams() returns the input and output directory info
//               and the analysis parameters.
//  -loadMatches() performs keypoint matching and returns a list
//                 of keypoints.
//  -init_tracks() calculates the disparity of each keypoint.
//  -computeMatXGradient() is an implementation of the matlab
//                         gradient function which uses central
//                         differences.
//  -init_arrays() initialize the input structures.
//  -startVA() starts cost volume filtering and disparity
//             estimation.
//
// Copyright (C) Mohamed Helala 2014
// All rights reserved
// Email: firstname.lastname@uoit.ca
//////////////////////////////////////////////////////////////////////

#ifndef FSMRF_H
#define FSMRF_H
#endif // FSMRF_H

#include <iostream>
#include <fstream>
#include <cv.h>
#include "opencv2/nonfree/features2d.hpp"
#include <highgui.h>
#include <math.h>
#include <SLIC.h>
#include <unistd.h>
#include <writeMat.h>
#include <util.h>
#include <params.h>
#include <volanalysis.h>

using namespace cv;

Track** loadMatches(Mat img_1, Mat img_2, int& nT);
Mat computeMatXGradient(const Mat &mat, int ddepth=CV_64F);
Params *getparams(int argc, char *argv[]);
Track** init_tracks(Mat I[], int &nT, Params *params);
void init_arrays(Mat I[], Mat In[], Mat Ilab[], Mat Igrad[], Mat Isp[], int nsp[], int in_nsp);
