//// volanalysis.cpp: implementation of the VolAnalysis class. ////
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

#include <volanalysis.h>
#include <time.h>
#include <vector>
#include  <util.h>

using namespace std;

//################# Public Methods

// starts the disparity estimation of the Cost Filter (CF) using
// displacements in the x direction.
void VolAnalysis::gen_disptr(Mat *I, Mat *Igrad, Mat &disp1, Mat &disp2){

    int vc = params->vc;
    Mat a = I[vc], b = I[1-vc],
            a_grad = Igrad[vc], b_grad = Igrad[1-vc];

    disp1 = Mat(a.rows, a.cols, CV_64F, Scalar::all(-1));
    Mat cost1 = Mat(a.rows, a.cols, CV_64F, Scalar::all(INT_MAX));
    disp2 = Mat(b.rows, b.cols, CV_64F, Scalar::all(-1));
    Mat cost2 = Mat(b.rows, b.cols, CV_64F, Scalar::all(INT_MAX));
    double diff=0;
    int numDisp = params->dispRng[1];
    Mat *dispVol1 = new Mat[numDisp];
    Mat *dispVol2 = new Mat[numDisp];

    if(!params->intermed_yaml || !params->read_yaml){
        for (int d=0; d<numDisp; d++){
            int dx = d+1;
            cout<<dx<<"- ";
            struct timeval t1 = timer_start();
            Mat c = compute_cost(a, b, a_grad, b_grad, dx, true);
            dispVol1[d] = filter_cost(a, c);
            diff +=timer_stop(t1);
            c = compute_cost(b, a, b_grad, a_grad, dx, false);
            dispVol2[d] = filter_cost(b, c);
            cout<<"Elasped time is "<<diff/1000<<" seconds."<<endl;
            cout.flush();
        }
    }
    if(!params->intermed_yaml || !params->read_yaml){
        // WTA
        wta(dispVol1, numDisp, cost1, disp1);
        wta(dispVol2, numDisp, cost2, disp2);

        if(params->show_stages){
            Mat m1, m2;
            convertScaleAbs(disp1, m1, params->scale);
            convertScaleAbs(disp2, m2, params->scale);
            namedWindow("disp-left", WINDOW_NORMAL);
            namedWindow("disp-right", WINDOW_NORMAL);
            imshow("disp-left", m1);
            imshow("disp-right", m2);
        }

        // consistency check
        check_consist(disp1, disp2, true);
        check_consist(disp1, disp2, false);
    }
    if(params->intermed_yaml && params->read_yaml){
        //read
        disp1 = ymlread("disp1.yml");
        disp2 = ymlread("disp2.yml");
    }else if(params->intermed_yaml && !params->read_yaml){
        //write
        ymlwrite(disp1, "disp1.yml");
        ymlwrite(disp2, "disp2.yml");
    }
    if(params->show_stages){
        Mat m1, m2;
        convertScaleAbs(disp1, m1, params->scale);
        convertScaleAbs(disp2, m2, params->scale);
        namedWindow("disp-left-gaps", WINDOW_NORMAL);
        namedWindow("disp-right-gaps", WINDOW_NORMAL);
        imshow("disp-left-gaps", m1);
        imshow("disp-right-gaps", m2);
    }
    cout<<"Total Elasped time is "<<diff/1000<<" seconds."<<endl;
    cout.flush();
}

// starts the disparity estimation of the Accelerated Cost volume Filtering
// (ACF) method using displacements in the x direction.
void VolAnalysis::gen_disptr_salient(Mat *I, Mat *Igrad, Track **T, int nT, Mat &disp1, Mat &disp2){

    int vc = params->vc;
    Mat a = I[vc], b = I[1-vc],
            a_grad = Igrad[vc], b_grad = Igrad[1-vc];

    disp1 = Mat(a.rows, a.cols, CV_64F, Scalar::all(-1));
    Mat cost1 = Mat(a.rows, a.cols, CV_64F, Scalar::all(INT_MAX));
    disp2 = Mat(b.rows, b.cols, CV_64F, Scalar::all(-1));
    Mat cost2 = Mat(b.rows, b.cols, CV_64F, Scalar::all(INT_MAX));
    double diff;
    int numDisp = params->dispRng[1];
    Mat *dispVol1 = new Mat[numDisp];
    Mat *dispVol2 = new Mat[numDisp];

    bool edisp[numDisp];
    VRect a_ir[numDisp];
    VRect a_eir[numDisp];
    //other view
    VRect b_ir[numDisp];
    VRect b_eir[numDisp];

    memset(edisp, 0, sizeof(bool)*numDisp);

    con_sal_wins(a_ir, b_ir, edisp, T, nT, vc, a.cols, a.rows);
    con_sal_subvols(a_ir, b_ir, a_eir, b_eir, edisp, numDisp, params->uw);

    if(!params->intermed_yaml || !params->read_yaml){
        for (int d=0; d<numDisp; d++){
            if(!edisp[d]) continue;
            int dx = d+1;
            cout<<dx<<"- ";
            struct timeval t1 = timer_start();
            Mat c = compute_cost(a, b, a_grad, b_grad, dx, true);
            dispVol1[d] = filter_cost(a, c, &a_eir[d]);
            diff+=timer_stop(t1);
            c = compute_cost(b, a, b_grad, a_grad, dx, false);
            dispVol2[d] = filter_cost(b, c, &b_eir[d]);
            cout<<"Elasped time is "<<diff/1000<<" seconds."<<endl;
            cout.flush();
        }
    }
    if(!params->intermed_yaml || !params->read_yaml){
        //WTA
        wta(dispVol1, numDisp, cost1, disp1, edisp);
        wta(dispVol2, numDisp, cost2, disp2, edisp);

        if(params->show_stages){
            Mat m1, m2;
            convertScaleAbs(disp1, m1, params->scale);
            convertScaleAbs(disp2, m2, params->scale);
            namedWindow("disp-left", WINDOW_NORMAL);
            namedWindow("disp-right", WINDOW_NORMAL);
            imshow("disp-left", m1);
            imshow("disp-right", m2);
        }
        // consistency check
        check_consist(disp1, disp2, true);
        check_consist(disp1, disp2, false);

    }

    if(params->intermed_yaml && params->read_yaml){
        //read
        disp1 = ymlread("disp1.yml");
        disp2 = ymlread("disp2.yml");
    }else if(params->intermed_yaml && !params->read_yaml){
        //write
        ymlwrite(disp1, "disp1.yml");
        ymlwrite(disp2, "disp2.yml");
    }
    if(params->show_stages){
        Mat m1, m2;
        convertScaleAbs(disp1, m1, params->scale);
        convertScaleAbs(disp2, m2, params->scale);
        namedWindow("disp-left-gaps", WINDOW_NORMAL);
        namedWindow("disp-right-gaps", WINDOW_NORMAL);
        imshow("disp-left-gaps", m1);
        imshow("disp-right-gaps", m2);
    }

    cout<<"Total Elasped time is "<<diff/1000<<" seconds."<<endl;
    cout.flush();
}

// performs superpixel based post-processing, inspired by simulated annealing.
void VolAnalysis::post_process(Mat I, Mat &disp, Mat Isp, int nsp, double occThr){

    int votes[nsp];
    double thresColor = 1/255.0;
    VRect *Rs = new VRect[nsp];
    // divide superpixels into occluded and non-occluded
    int n_occpixs[nsp];
    bool occ[nsp];
    vector<int> Rocc;

    memset(n_occpixs, 0, sizeof(int)*nsp);
    memset(occ, 0, sizeof(bool)*nsp);
    memset(votes, -1, sizeof(int)*nsp);

    gen_recs(I, Isp, nsp, Rs);
    gen_graph(Isp, nsp, Rs);

    gen_votes(Rs, disp, Isp, nsp, votes, n_occpixs);

    // determine the occ superpixels with
    for(int s=0; s<nsp; s++){
        double occFreq = ((double)n_occpixs[s])/Rs[s].npix;
        if(occFreq>occThr){
            Rocc.push_back(s);
            occ[s] = true;
        }else{
            // fill regions with occFreq<=occThr using their own votes.
            if(occFreq!=0){
                for(int x=Rs[s].x1;x<=Rs[s].x2; x++){
                    for(int y=Rs[s].y1;y<=Rs[s].y2; y++){
                        if(disp.at<double>(y, x)==-1 && Isp.at<int>(y, x)==s){
                            disp.at<double>(y,x) = votes[s];
                        }
                    }
                }
                n_occpixs[s] = 0;
            }
        }
    }

    // iteratively fill the occluded regions if very similar
    fill_gaps(Rs, disp, Isp, votes, Rocc, occ, thresColor);
}

//#################### utilities

// The function performs a simulated annealing inspired strategy
// to fill occluded regions. The function fills each occluded
// region by the vote of its best similar region. Color similarity
// is used and the temperature variable controls the annealing.
void VolAnalysis::fill_gaps(VRect *Rs, Mat &disp, Mat Isp, int votes[], vector<int> Rocc, bool occ[], double temperature){
    bool entered = true;
    do{
        entered = false;
        bool progress = true;
        do{
            progress = false;
            for(int i=0; i<Rocc.size(); i++){
                int s = Rocc[i];
                if(!occ[s]) continue;
                entered = true;
                // find the most similar neighbor
                // given that the color difference < threshcolor
                VRect *cr = &Rs[s], *nn = 0;
                double dbest = INT_MAX;
                for(int j=0; j<cr->nes; j++){
                    int nnid = cr->es[j];
                    double dc = sqrt(pow(Rs[nnid].mr - cr->mr,2) +
                            pow(Rs[nnid].mg - cr->mg,2) +
                            pow(Rs[nnid].mb - cr->mb,2));
                    if(dc<dbest && !occ[nnid] && votes[nnid]!=-1){
                        dbest = dc; nn = &Rs[nnid];
                    }
                }

                if(dbest>temperature) continue;

                for(int x=cr->x1;x<=cr->x2; x++){
                    for(int y=cr->y1;y<=cr->y2; y++){
                        if(disp.at<double>(y, x)==-1 && Isp.at<int>(y, x)==s){
                            disp.at<double>(y,x) = votes[nn->id];
                        }
                    }
                }
                occ[s] = false;
                votes[s] = votes[nn->id];
                progress = true;
            }
        }while(progress);
        temperature = MIN(temperature+0.0001, 1);
    }while(entered);
}

// this method computes the cost of a slice dx in the cost
// volume (x, y, dx).
Mat VolAnalysis::compute_cost(Mat a, Mat b, Mat a_grad, Mat b_grad, int dx, bool move_left) {
    // Truncated SAD of color images for current displacement
    Mat tmp = Mat(a.rows, a.cols, a.type(), Scalar::all(THBorder));
    if (move_left) {
        b(Range(0, b.rows),Range(0, b.cols-dx))
                .copyTo(tmp(Range(0,a.rows), Range(dx,a.cols)));
    }else{
        b(Range(0, b.rows),Range(dx, b.cols))
                .copyTo(tmp(Range(0,a.rows), Range(0,a.cols-dx)));
    }

    Mat p_color1 = abs(tmp - a);

    vector<Mat> Ch1;
    split(p_color1, Ch1);
    Mat Dcolor1 = (Ch1[0] + Ch1[1] + Ch1[2])*0.333333333333;
    min(Dcolor1, THCOLOR, Dcolor1);

    // Truncated SAD of gradient images for current displacement
    Mat tmp1 = Mat(b_grad.rows, b_grad.cols, b_grad.type(), Scalar::all(THBorder));
    if (move_left){
        b_grad(Range(0, b.rows),Range(0, b.cols-dx))
            .copyTo(tmp1(Range(0,a.rows), Range(dx,a.cols)));
    }else{
        b_grad(Range(0, b.rows),Range(dx, b.cols))
                .copyTo(tmp1(Range(0,a.rows), Range(0,a.cols-dx)));
    }
    Mat Dgrad1= abs(tmp1 - a_grad);
    min(Dgrad1, THGRAD, Dgrad1);

    return GAMMA*Dcolor1 + (1-GAMMA)*Dgrad1;
}


// performs guided filtering of the given cost slice d using the
// guided image a. roi specifies a region of interest and is set
// to NULL to filter the entire d slice.
Mat VolAnalysis::filter_cost(Mat a, Mat d, VRect *roi){

    Mat a_roi = a, d_roi = d, dc;

    if(roi!=NULL){
        dc = d.clone();
        dc = dc +1;
        a_roi = a(Range(roi->y1, roi->y2+1), Range(roi->x1, roi->x2+1));
        d_roi = dc(Range(roi->y1, roi->y2+1), Range(roi->x1, roi->x2+1));
        d_roi = d_roi-1;
    }

    vector<Mat> aCh;
    split(a_roi, aCh);
    Mat p = guidedfilter(aCh[0], d_roi, params->r, params->eps);
    p = p + guidedfilter(aCh[1], d_roi, params->r, params->eps);
    p = p + guidedfilter(aCh[2], d_roi, params->r, params->eps);

    if (roi!=NULL){
        p.copyTo(d_roi);
        return dc;
    }

    return p;
}

// performs Winner-Takes-ALL (WTA) to select the disparity with
// minimum cost for each (x, y) location in the cost volume
// (x, y, dx).
void VolAnalysis::wta(Mat* vol, int n, Mat &cost, Mat &disp, bool edisp[]){
    for(int i=0; i<n; i++){
        if(edisp != NULL && !edisp[i]) continue;
        double dx = i+1;
        for (int x=0; x< vol[i].cols; x++){
            for (int y=0; y< vol[i].rows; y++){
                if(vol[i].at<double>(y, x)< cost.at<double>(y, x)){
                    disp.at<double>(y, x) = dx;
                    cost.at<double>(y, x) = vol[i].at<double>(y, x);
                }
            }
        }
    }

}

// detects regions of mismatched disparity (gaps) in either the
// left disp1 or right disp2 disparity labeling. The left_to_right
// bool variable controls which disparity will be checked. The
// mismatched disparity found at a location (x, y) is set to -1.
void VolAnalysis::check_consist(Mat &disp1, Mat &disp2, bool left_to_right){
    if (left_to_right){
        for(int x = 0; x<disp1.cols; x++){
            for(int y = 0; y<disp1.rows; y++){
                int xb = x - disp1.at<double>(y, x);
                int yb = y;
                if(xb>=0 && xb<disp2.cols && yb>=0 && yb<disp2.rows){
                    if(abs(disp2.at<double>(yb, xb)- disp1.at<double>(y, x))>=1){
                        disp1.at<double>(y, x) = -1;
                    }
                }else{
                    disp1.at<double>(y, x) = -1;
                }
            }
        }
    }else{
        for(int x = 0; x<disp2.cols; x++){
            for(int y = 0; y<disp2.rows; y++){
                int xa = x + disp2.at<double>(y, x);
                int ya = y;
                if(xa>=0 && xa<disp1.cols && ya>=0 && ya<disp1.rows){
                    if(abs(disp1.at<double>(ya, xa)- disp2.at<double>(y, x))>=1){
                        disp2.at<double>(y, x) = -1;
                    }
                }else{
                    disp2.at<double>(y, x) = -1;
                }
            }
        }
    }
}

// generate a rectangular envelope for each superpixel.
// see VRect struct definition for details.
void VolAnalysis::gen_recs(Mat I, Mat Isp, int nsp, VRect *Rs){
    for (int y=0;y<I.rows; y++){
        for (int x=0;x<I.cols; x++){
            int id = Isp.at<int>(y, x);
            VRect* rec = &Rs[id];
            if (rec->x2<x) rec->x2 = x;
            if (rec->y2<y) rec->y2 = y;
            if (rec->x1>x) rec->x1 = x;
            if (rec->y1>y) rec->y1 = y;
            rec->id = id;
            rec->npix++;
            // Sum region color.
            Vec3d c = I.at<Vec3d>(y, x);
            rec->mr += c[0]; rec->mg += c[1]; rec->mb += c[2];
        }
    }
    for (int b = 0; b < nsp; b++) {
        VRect *c_rec = &Rs[b];
        c_rec->es = new int[20];
        c_rec->mr = c_rec->mr/c_rec->npix;
        c_rec->mg = c_rec->mg/c_rec->npix;
        c_rec->mb = c_rec->mb/c_rec->npix;
    }
}

// generate a superpixel graph (S, <S,S>) with vertices as superpixels
// and edges define neighboring superpixels.
void VolAnalysis::gen_graph(Mat Isp, int nsp, VRect* Rc){

    int xc, yc;
    for(int k=0; k<nsp; k++){
        VRect *r = &Rc[k];
        for(int x=r->x1; x<=r->x2; x++){
            for(int y=r->y1; y<=r->y2; y++){
                int cur=-1, oth=-1;
                for(int i=-1; i<2;i++){
                    for(int j=-1; j<2;j++){
                        yc = y+j; xc = x+i;
                        if(yc<0 || yc>=Isp.rows || xc<0 || xc>= Isp.cols) continue;
                        int nid  = Isp.at<int>(yc, xc);
                        if(nid == r->id)cur=r->id;
                        if(nid != r->id)oth=nid;
                    }
                }
                if(cur!=-1 && oth!=-1){
                    bool sel=false;
                    for (int i=0; i<r->nes; i++) if(oth == r->es[i]) sel= true;
                    if(sel) continue;
                    r->es[r->nes] = oth;
                    r->nes++;
                }
            }
        }
    }

}

// generates a voted disparity for each superpixel. Also set the
// number of occluded pixels n_occpixs in each superpixels.
void VolAnalysis::gen_votes(VRect *Rs, Mat disp, Mat Isp, int nsp, int votes[], int n_occpixs[]){
    vector<double> cdisps;
    for(int s=0; s<nsp; s++){
        for(int x = Rs[s].x1; x<=Rs[s].x2; x++){
            for(int y = Rs[s].y1; y<=Rs[s].y2; y++){
                if(!(Isp.at<int>(y, x)==s)) continue;
                if(disp.at<double>(y, x)==-1){
                    n_occpixs[s]++;
                }else{
                    cdisps.push_back(disp.at<double>(y,x));
                }
            }
        }
        if(cdisps.size()==0) {continue;}
        int n = MAX(n_occpixs[s]*0.5, 1);
        int _v = 0, _k = -1;
        for(int i=0; i<n; i++){
            int x = ((double) rand() / (RAND_MAX))*cdisps.size();
            int k = 0;
            for(int j=0; j<cdisps.size(); j++){
                if (j != x){
                    if (abs(cdisps[x]-cdisps[j])<0.5){
                       k++;
                    }
                }
            }
            if (k>_k){
                _k = k;
                _v = cdisps[x];
            }
        }
        if(_k!=-1) votes[s] = _v;
        cdisps.clear();
    }
}

// constructs a salient window in each cost slice. This window
// results from the union of local windows defined arround the
// keypoints that have similar disparity to the one of the
// input slice.
void VolAnalysis::con_sal_wins(VRect a_ir[], VRect b_ir[], bool edisp[], Track **T, int nT, int vc, int w, int h){
    int rw = w*params->rs, rh =h*params->rs;
    for(int t=0; t< nT; t++){
        int l = round(T[t]->dispx)-1;
        int ax  = round(T[t]->pts[vc].x), ay = round(T[t]->pts[vc].y);
        int ax1 = MAX(ax-rw-(params->r+1), 0), ay1 = MAX(ay-rh-(params->r+1), 0);
        int ax2 = MIN(ax+rw+(params->r+1), w-1), ay2 = MIN(ay+rh+(params->r+1), h-1);
        // other view
        int bx  = round(T[t]->pts[1-vc].x), by = round(T[t]->pts[1-vc].y);
        int bx1 = MAX(bx-rw-(params->r+1), 0), by1 = MAX(by-rh-(params->r+1), 0);
        int bx2 = MIN(bx+rw+(params->r+1), w-1), by2 = MIN(by+rh+(params->r+1), h-1);

        if(!edisp[l]){
            edisp[l] = true;
            a_ir[l].x1 = ax1; a_ir[l].x2 = ax2; a_ir[l].y1 = ay1; a_ir[l].y2 = ay2;
            b_ir[l].x1 = bx1; b_ir[l].x2 = bx2; b_ir[l].y1 = by1; b_ir[l].y2 = by2;
        }else{
            // union
            a_ir[l] = a_ir[l]._union(ax1, ay1, ax2, ay2);
            b_ir[l] = b_ir[l]._union(bx1, by1, bx2, by2);
        }
    }
}

// construct salient subvolumes by defining the final salient
// window of each slice l as the union of salient windows of
// neighboring slices |l' - l|<=uw.
void VolAnalysis::con_sal_subvols(VRect a_ir[], VRect b_ir[], VRect a_eir[], VRect b_eir[], bool edisp[], int numDisp, int uw){

    vector<int> idxs;
    for(int i=0;i<numDisp; i++){
        if(edisp[i]){idxs.push_back(i);}
    }

    for(int i=0;i<idxs.size(); i++){
        int l = idxs[i];
        a_eir[l] = a_ir[l];
        b_eir[l] = b_ir[l];
        for(int j=i-uw; j<i+uw; j++){
            if(j!=i && j>=0 && j<idxs.size()){
                // union
                a_eir[l] = a_eir[l]._union(a_ir[idxs[j]]);
                b_eir[l] = b_eir[l]._union(b_ir[idxs[j]]);
            }
        }
    }

    for(int i=0, c=0, prev=-1;i<numDisp; i++){
        if(i==idxs[c]){
            prev = c++;
        }else{
            if(prev==-1 && (idxs[c]-i <= uw)){
                b_eir[i] = b_eir[idxs[c]];
                a_eir[i] = a_eir[idxs[c]];
                edisp[i]= true;
            }else if(prev==idxs.size()-1 && (i-idxs[prev] <= 2*uw)){
                b_eir[i] = b_eir[idxs[prev]];
                a_eir[i] = a_eir[idxs[prev]];
                edisp[i]= true;
            }else if(prev!=-1 && prev!=idxs.size()-1 && (abs(i-idxs[prev] <= uw))){
                b_eir[i] = b_eir[idxs[prev]]._union(b_eir[idxs[c]]);
                a_eir[i] = a_eir[idxs[prev]]._union(a_eir[idxs[c]]);
                edisp[i]= true;
            }
        }
    }
}
