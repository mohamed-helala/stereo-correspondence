//// volanalysis.h: Definition of the VolAnalysis class. ////
//   -gen_disptr() starts the disparity estimation of the Cost Filter (CF).
//   -gen_disptr_salient() starts the disparity estimation of the
//                         Accelerated Cost volume Filtering (ACF) method.
//   -post_process() performs superpixel based post-processing.
//
//   -other private functions are used internally. Please refer to the
//    comments of each function.
//
// Copyright (C) Mohamed Helala 2014
// All rights reserved
// Email: firstname.lastname@uoit.ca
//////////////////////////////////////////////////////////////////////

#ifndef VOLANALYSIS_H
#define VOLANALYSIS_H
#endif // VOLANALYSIS_H

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <cmath>
#ifndef PARAMS_H
#include <params.h>
#endif
#ifndef _KDTREE_H_
#include <kdtree.h>
#endif
#ifndef GUIDEDFILTER_H
#include <guidedfilter.h>
#endif


struct Track{
    Point2d* pts;
    double dispx=0;
    double dispy=0;
};

// this struct defines a rectangular envelope
// of a superpixel.
struct VRect {
  // rectangle
  int x1, y1, x2, y2;
  // mean (r,g,b) color of a uperpixel
  double mr, mg, mb;
  // number of neigboring superpixels.
  int nes;
  // pointers to neigboring superpixels.
  int *es;
  // number of pixels in the superpixel.
  int npix;
  // id of the superpixel (set to its index).
  int id;

  VRect():x1(INT_MAX), y1(INT_MAX), x2(0), y2(0),
      mr(-1), mg(-1), mb(-1), nes(0), npix(0), id(-1){

  }

  VRect intesect(VRect r){
      VRect ri;
      ri.x1 = MAX(this->x1, r.x1);
      ri.x2 = MIN(this->x2, r.x2);
      ri.y1 = MAX(this->y1, r.y1);
      ri.y2 = MIN(this->y2, r.y2);
      return ri;
  }

  VRect _union(VRect r){
    VRect ri;
    ri.x1 = MIN(this->x1, r.x1);
    ri.x2 = MAX(this->x2, r.x2);
    ri.y1 = MIN(this->y1, r.y1);
    ri.y2 = MAX(this->y2, r.y2);
    return ri;
  }

  VRect _union(int x1, int y1, int x2, int y2){
    VRect ri;
    ri.x1 = MIN(this->x1, x1);
    ri.x2 = MAX(this->x2, x2);
    ri.y1 = MIN(this->y1, y1);
    ri.y2 = MAX(this->y2, y2);
    return ri;
  }
} ;

class VolAnalysis{
public:
    Params *params;
    double THBorder;
    double GAMMA;
    double THCOLOR;
    double THGRAD;

    VolAnalysis(Params *_params):params(_params),THBorder(3.0/255.0),
        GAMMA(0.11),THCOLOR(7.0/255.0),THGRAD(2.0/255.0){}

    void gen_disptr(Mat *I, Mat *Igrad, Mat &disp1, Mat &disp2);
    void gen_disptr_salient(Mat *I, Mat *Igrad, Track **T, int nT, Mat &disp1, Mat &disp2);
    void post_process(Mat I, Mat &disp, Mat Isp, int nsp, double occThr);

private:

    void fill_gaps(VRect *Rs, Mat &disp, Mat Isp, int votes[], vector<int> Rocc, bool occ[], double temperature);

    Mat compute_cost(Mat a, Mat b, Mat a_grad, Mat b_grad, int dx, bool move_left);
    Mat filter_cost(Mat a, Mat d, VRect *roi = NULL);
    void wta(Mat* vol, int n, Mat &cost, Mat &disp, bool edisp[] = NULL);
    void check_consist(Mat &disp1, Mat &disp2, bool left_to_right);
    void gen_votes(VRect *Rs, Mat disp, Mat Isp, int nsp, int votes[], int n_occpixs[]);

    void con_sal_wins(VRect a_ir[], VRect b_ir[], bool edisp[], Track **T, int nT, int vc, int w, int h);
    void con_sal_subvols(VRect a_ir[], VRect b_ir[], VRect a_eir[], VRect b_eir[], bool edisp[], int numDisp, int uw);

    void gen_recs(Mat I, Mat Isp, int nsp, VRect *Rs);
    void gen_graph(Mat Isp, int nsp, VRect* Rc);

};
