//// util.cpp: implementation of utility functions. ////
//   -ymlwrite() writes the given Mat in a yml file.
//   -ymlread() reads a Mat from the given yml file.
//   -timer_start() returns the current time.
//   -timer_stop() returns the elapsed seconds between.
//                 the current time and the given time.
//
// Copyright (C) Mohamed Helala 2014
// All rights reserved
// Email: firstname.lastname@uoit.ca
//////////////////////////////////////////////////////////////////////

#include<util.h>

// writes the given Mat in a yml file s.
void ymlwrite(Mat m, string s){
    FileStorage fs(s, FileStorage::WRITE);
    fs << "Matrix" << m;
    fs.release();
}

// reads a Mat from the given yml file s.
Mat ymlread(string s){
    FileStorage fs(s, FileStorage::READ);
    Mat m;
    fs["Matrix"] >> m;
    fs.release();
    return m;
}

// returns the current time.
struct timeval timer_start()
{
    struct timeval tm1;
    gettimeofday(&tm1, NULL);
    return tm1;
}

// returns the time difference (in seconds) between
// the current time and t.
unsigned long long timer_stop(struct timeval t)
{
    struct timeval tm2;
    gettimeofday(&tm2, NULL);

    return 1000 * (tm2.tv_sec - t.tv_sec) + (tm2.tv_usec - t.tv_usec) / 1000;
}
