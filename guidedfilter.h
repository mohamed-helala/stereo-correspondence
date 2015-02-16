//// guidedfilter.cpp: implementation of the guided filter method. ////
//   -cumsum() returns the cummulative sum over x and y for radius r.
//   -boxfilter() implements the boxfilter
//   -guidedfilter() o(1) implementation of the guided filter.
//   -test_guidedfltr() testing functions for the guided filter.
//
// Copyright (C) Mohamed Helala 2014
// All rights reserved
// Email: firstname.lastname@uoit.ca
//////////////////////////////////////////////////////////////////////

#ifndef GUIDEDFILTER_H
#define GUIDEDFILTER_H

#endif // GUIDEDFILTER_H

#include <iostream>
#include <cv.h>
#include <highgui.h>
#include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

Mat cumsum(Mat src,int rc);
Mat boxfilter1(Mat src,int rc);
Mat guidedfilter(Mat I, Mat p, int r, double eps);
void test_guidedfltr(Mat image);
void test_guidedfltr(Mat simage, Mat gimage);
