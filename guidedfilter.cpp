//// guidedfilter.cpp: implementation of the guided filter method. ////
//   -cumsum() returns the cummulative sum over x and y for radius r.
//   -boxfilter() implements the boxfilter.
//   -guidedfilter() o(1) implementation of the guided filter.
//   -test_guidedfltr() testing functions for the guided filter.
//
// Copyright (C) Mohamed Helala 2014
// All rights reserved
// Email: firstname.lastname@uoit.ca
//////////////////////////////////////////////////////////////////////

#include<guidedfilter.h>

// returns the cummulative sum over x and y for radius r
Mat cumsum(Mat src,int r)
{
    Mat cum = src.clone();
    if (r==1){
        for(int y=1;y<src.rows;y++){
            for(int x=0;x<src.cols;x++){
                cum.at<double>(y, x) = cum.at<double>(y-1, x) + cum.at<double>(y, x);
            }
        }
    }
    if (r==2){
        for(int x=1;x<src.cols;x++){
            for(int y=0;y<src.rows;y++){
                cum.at<double>(y, x) = cum.at<double>(y, x-1) + cum.at<double>(y, x);
            }
        }
    }
    return cum;
}

// implements the boxfilter
Mat boxfilter(Mat imSrc,int r)
{
    int hei = imSrc.rows, wid = imSrc.cols;
    Mat imDst = Mat::zeros(hei, wid, imSrc.type());

    //cumulative sum over Y axis
    Mat imCum = cumsum(imSrc, 1);

    //difference over Y axis
    imCum(Range(r,2*r+1), Range(0,wid)).copyTo(imDst(Range(0,r+1), Range(0,wid)));
    Mat tmp = imCum(Range(2*r+1,hei), Range(0,wid)) - imCum(Range(0,hei-2*r-1), Range(0,wid));
    if(!tmp.empty())
        tmp.copyTo(imDst(Range(r+1,hei-r), Range(0,wid)));
    tmp = repeat(imCum(Range(hei-1,hei), Range(0,wid)),r,1) - imCum(Range(hei-2*r-1, hei-r-1), Range(0,wid));
    if(!tmp.empty())
        tmp.copyTo(imDst(Range(hei-r,hei), Range(0, wid)));

    //cumulative sum over X axis
    imCum = cumsum(imDst, 2);

    //difference over Y axis
    imCum(Range(0,hei), Range(r,2*r+1)).copyTo(imDst(Range(0,hei), Range(0,r+1)));
    tmp = imCum(Range(0,hei), Range(2*r+1,wid)) - imCum(Range(0,hei), Range(0,wid-2*r-1));
    if(!tmp.empty())
        tmp.copyTo(imDst(Range(0,hei), Range(r+1,wid-r)));
    tmp = repeat(imCum(Range(0,hei), Range(wid-1,wid)),1,r) - imCum(Range(0, hei), Range(wid-2*r-1,wid-r-1));
    if(!tmp.empty())
        tmp.copyTo(imDst(Range(0,hei), Range(wid-r, wid)));

    return imDst;
}

//   GUIDEDFILTER   O(1) time implementation of guided filter.
//
//   - guidance image: I (should be a gray-scale/single channel image)
//   - filtering input image: p (should be a gray-scale/single channel image)
//   - local window radius: r
//   - regularization parameter: eps
Mat guidedfilter(Mat I, Mat p, int r, double eps){

    Mat mean_I= I.clone(), mean_p= p.clone();
    Mat mean_Ip = I.clone(), tmp = I.clone();
    Mat mean_II = I.clone(), a = I.clone();
    Mat b = I.clone(), mean_a = I.clone(),
            mean_b = I.clone();

    // The size of each local patch; N=(2r+1)^2 except for boundary pixels.
    Mat N = boxfilter(Mat::ones(I.rows, I.cols, I.type()), r);

    divide(boxfilter(I, r), N, mean_I);

    divide(boxfilter(p, r), N, mean_p);

    multiply(I, p, tmp);
    divide(boxfilter(tmp, r), N, mean_Ip);

    multiply(mean_I, mean_p, tmp);
    // This is the covariance of (I, p) in each local patch.
    Mat cov_Ip = mean_Ip - tmp;

    multiply(I, I, tmp);
    divide(boxfilter(tmp, r), N, mean_II);

    multiply(mean_I, mean_I, tmp);
    Mat var_I = mean_II - tmp;

    //Eqn. (5) in the paper;
    divide(cov_Ip, (var_I + eps), a);

    //Eqn. (6) in the paper;
    multiply(a, mean_I, tmp);
    b = mean_p - tmp;

    divide(boxfilter(a, r), N, mean_a);
    divide(boxfilter(b, r), N, mean_b);

    //Eqn. (8) in the paper;
    multiply(mean_a, I, tmp);
    Mat q =  tmp + mean_b;

    return q;
}

// tests the guided filter
void test_guidedfltr(Mat image){

    Mat channel[3];
    Mat channelp[3];
    Mat channelq[3];
    Mat _image = image.clone();
    int r = 16;
    double eps = pow(0.1,2.0);

    split(_image,channel);
    for(int i=0; i<_image.channels(); i++){
        channel[i].convertTo(channelp[i],CV_64F,1.0/255.0);
        channelq[i] = guidedfilter(channelp[i], channelp[i], r, eps);
        channelq[i] = (channelp[i] - channelq[i])*5 + channelq[i];
        channelq[i].convertTo(channel[i], channel[i].type(), 255);
    }

    merge(channel,3,_image);
    imshow("input", _image);
}

// tests the guided filter
void test_guidedfltr(Mat simage, Mat gimage){

    Mat channel[3];
    Mat channelp[3];
    Mat channelq[3];
    Mat _gimage;
    Mat _simage = simage.clone();
    int r = 16;
    double eps = pow(0.1,2.0);

    split(_simage,channel);
    for(int i=0; i<_simage.channels(); i++){
        channel[i].convertTo(channelp[i],CV_64F,1.0/255.0);
        gimage.convertTo(_gimage, CV_64F,1.0/255.0);
        channelq[i] = guidedfilter(channelp[i], _gimage, r, eps);
        channelq[i] = (channelp[i] - channelq[i])*5 + channelq[i];
        channelq[i].convertTo(channel[i], channel[i].type(), 255);
    }

    merge(channel,3,_simage);
    imshow("guided filter test", _simage);
}
