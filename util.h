//// util.cpp: implementation of utility functions. ////
//   -ymlwrite() writes the given Mat in a yml file.
//   -ymlread() reads a Mat from the given yml file.
//   -timer_start() returns the current time.
//   -timer_stop() takes a previous time and returns the elapsed
//                 seconds.
//
// Copyright (C) Mohamed Helala 2014
// All rights reserved
// Email: firstname.lastname@uoit.ca
//////////////////////////////////////////////////////////////////////

#ifndef UTIL_H
#define UTIL_H

#endif // UTIL_H
#include <sys/time.h>
#ifndef __OPENCV_OLD_CV_H__
#include <cv.h>
#endif
#ifndef __OPENCV_OLD_HIGHGUI_H__
#include <highgui.h>
#endif

using namespace cv;

struct timeval timer_start();
unsigned long long timer_stop(timeval t);
void ymlwrite(Mat m, string s);
Mat ymlread(string s);
