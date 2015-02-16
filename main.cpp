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

#include <main.h>

using namespace std;
using namespace cv;

// start volume analysis.
int startVA(Params *params)
{
    cout << "Using OpenCV " << CV_MAJOR_VERSION << "." << CV_MINOR_VERSION << "." << CV_SUBMINOR_VERSION << std::endl;
    Mat I[2], In[2], Ilab[2], Igrad[2], Isp[2];
    int nsp[2];
    int vc = params->vc;
    int nT=0;
    stringstream ss;

    I[0] = imread(params->left_image);
    I[1] = imread(params->right_image);

    Track** T = init_tracks(I, nT, params);
    init_arrays(I, In, Ilab, Igrad, Isp, nsp, params->nsp);

    VolAnalysis *gc = new VolAnalysis(params);
    Mat disp1, disp2, m1, m2;

    if(params->calc == Calc_VolumeFiltering)
        gc->gen_disptr(In, Igrad, disp1, disp2);
    else if(params->calc == Calc_VolumeFiltering_Salient)
        gc->gen_disptr_salient(In, Igrad, T, nT, disp1, disp2);

    if(!disp1.empty()){
        if(params->all_results){
            ss.str(String()); ss<<string(params->outdirectory)<<"/"<<"im0occ"<<string(params->suffix)<<".mat";
            writeMat(disp1, ss.str().c_str(), (const char*)"A", false);
        }
        gc->post_process(In[vc], disp1,  Isp[vc], nsp[vc], params->occlThr);
        if(params->show_stages){
            convertScaleAbs(disp1,m1,params->scale);
            namedWindow("disp-left-postprocess", WINDOW_NORMAL);
            imshow("disp-left-postprocess", m1);
        }
        if(params->all_results || params->main_results){
            ss.str(String()); ss<<string(params->outdirectory)<<"/"<<"im0nocc"<<string(params->suffix)<<".mat";
            writeMat(disp1, ss.str().c_str(), (const char*)"B", false);
        }
    }
    if(!disp2.empty()){
        if(params->all_results){
            ss.str(String()); ss<<string(params->outdirectory)<<"/"<<"im1occ"<<string(params->suffix)<<".mat";
            writeMat(disp2, ss.str().c_str(), (const char*)"C", false);
        }
        gc->post_process(In[1-vc], disp2,  Isp[1-vc], nsp[1-vc], params->occlThr);
        if(params->show_stages){
            convertScaleAbs(disp2,m2,params->scale);
            namedWindow("disp-right-postprocess", WINDOW_NORMAL);
            imshow("disp-right-postprocess", m2);
        }
        if(params->all_results || params->main_results){
            ss.str(String()); ss<<string(params->outdirectory)<<"/"<<"im1nocc"<<string(params->suffix)<<".mat";
            writeMat(disp2, ss.str().c_str(), (const char*)"D", false);
        }
    }

    if(params->show_stages)
        waitKey(0);
    if((!disp1.empty() || !disp2.empty()) && params->show_stages)
        cvDestroyAllWindows();

    return 0;
}

// entry point.
int main(int argc, char *argv[]){
    Params *params = getparams(argc, argv);
    startVA(params);
}

// perform keypoint matching
Track** loadMatches(Mat img_1, Mat img_2, int& nt){

    if( !img_1.data || !img_2.data )
    { std::cout<< " --(!) Error reading images " << std::endl; return NULL; }

    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 200;

    SurfFeatureDetector detector( minHessian );

    std::vector<KeyPoint> keypoints_1, keypoints_2;

    detector.detect( img_1, keypoints_1 );
    detector.detect( img_2, keypoints_2 );

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;

    Mat descriptors_1, descriptors_2;

    extractor.compute( img_1, keypoints_1, descriptors_1 );
    extractor.compute( img_2, keypoints_2, descriptors_2 );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );

    double max_dist = 0; double min_dist = 250;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
    }

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;

    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance < 5*min_dist )
        { good_matches.push_back( matches[i]); }
    }
    nt = good_matches.size();
    Track** tracks= new Track*[nt];
    for(int i=0; i<nt; i++){
        tracks[i] = new Track();
        tracks[i]->pts = new Point2d[2];
        Point2f p1 = keypoints_1[good_matches[i].queryIdx].pt;
        Point2f p2 = keypoints_2[good_matches[i].trainIdx].pt;
        tracks[i]->pts[0] = Point2d(p1.x, p2.y);
        tracks[i]->pts[1] = Point2d(p2.x, p2.y);
    }
    cout<<"found "<<nt<< " keypoints"<<endl;
    return tracks;
}

// returns volume analysis parameters plus input output files and
// directories.
Params *getparams(int argc, char *argv[]){
    Params *parms = new Params();
    parms->show_stages = true;
    parms->debug = false;
    parms->all_results = true;
    parms->main_results = false;

    int seq = atoi(argv[1]);
    int nsp = (argc>2)?atoi(argv[2]):-1;
    double occlThr = (argc>3)?atof(argv[3]):0.6;
    double rs = (argc>4)?atof(argv[4]):-1;
    int salient = (argc>5)?atoi(argv[5]):-1;
    parms->calc =(salient>0)?Calc_VolumeFiltering_Salient:Calc_VolumeFiltering;

    if(argc>6){
        parms->intermed_yaml = atoi(argv[6]);
        parms->read_yaml = atoi(argv[7]);
    }else{
        parms->intermed_yaml=false;
    }
    parms->suffix = "";

    if(seq==0){
        parms->indirectory = "data/cones/";
        parms->outdirectory = "results/cones/";
        parms->left_image = "data/cones/00.png";
        parms->right_image = "data/cones/01.png";
        parms->nsp = (nsp>0)?nsp:1600;
        parms->vc = 0;
        parms->scale = 4;
        parms->r = 5;
        parms->eps = 0.0001;
        parms->occlThr = (occlThr>0)?occlThr:0.6;
        parms->rs = (rs>0)?rs:0.3;
        parms->uw = 4;
        parms->occlThr = 0.6;
        parms->dispRng[0] = 1; parms->dispRng[1] = 59;
    } else if(seq==1){
        parms->indirectory = "data/teddy/";
        parms->outdirectory = "results/teddy/";
        parms->left_image = "data/teddy/00.png";
        parms->right_image = "data/teddy/01.png";
        parms->nsp = (nsp>0)?nsp:2000;
        parms->vc = 0;
        parms->rs = (rs>0)?rs:0.3;
        parms->eps = 0.00001;
        parms->r = 5;
        parms->occlThr = (occlThr>0)?occlThr:0.6;
        parms->scale = 4;
        parms->uw = 4;
        parms->dispRng[0] = 1; parms->dispRng[1] = 59;
    } else if(seq==2){
        parms->indirectory = "data/venus/";
        parms->outdirectory = "results/venus/";
        parms->left_image = "data/venus/00.png";
        parms->right_image = "data/venus/01.png";
        parms->occlThr = (occlThr>0)?occlThr:0.6;
        parms->nsp = (nsp>0)?nsp:1000;
        parms->scale = 8;
        parms->eps = 0.00001;
        parms->rs = (rs>0)?rs:0.3;
        parms->r = 5;
        parms->vc = 0;
        parms->uw = 4;
        parms->dispRng[0] = 1; parms->dispRng[1] = 32;
    } else if(seq==3){
        parms->indirectory = "data/Tsukuba/";
        parms->outdirectory = "results/Tsukuba/";
        parms->left_image = "data/Tsukuba/00.png";
        parms->right_image = "data/Tsukuba/01.png";
        parms->occlThr = (occlThr>0)?occlThr:0.5;
        parms->nsp = (nsp>0)?nsp:500;
        parms->scale = 16;
        parms->eps = 0.001;
        parms->rs = (rs>0)?rs:0.3;
        parms->r = 5;
        parms->vc = 0;
        parms->uw = 4;
        parms->dispRng[0] = 1; parms->dispRng[1] = 15;
    } else if(seq==4){
        parms->indirectory = "data/2005/Baby3/";
        parms->outdirectory = "results/2005/Baby3/";
        parms->left_image = "data/2005/Baby3/00.png";
        parms->right_image = "data/2005/Baby3/01.png";
        parms->occlThr = (occlThr>0)?occlThr:0.5;
        parms->nsp = (nsp>0)?nsp:700;
        parms->scale = 1;
        parms->eps = 0.0001;
        parms->rs = (rs>0)?rs:0.3;
        parms->r = 9;
        parms->vc = 0;
        parms->uw = 2;
        parms->dispRng[0] = 1; parms->dispRng[1] = 230;
    }else if(seq==5){
        parms->indirectory = "data/2005/Rocks2/";
        parms->outdirectory = "results/2005/Rocks2/";
        parms->left_image = "data/2005/Rocks2/00.png";
        parms->right_image = "data/2005/Rocks2/01.png";
        parms->occlThr = (occlThr>0)?occlThr:0.5;
        parms->nsp = (nsp>0)?nsp:700;
        parms->scale = 1;
        parms->eps = 0.0001;
        parms->rs = (rs>0)?rs:0.2;
        parms->r = 9;
        parms->vc = 0;
        parms->uw = 2;
        parms->dispRng[0] = 1; parms->dispRng[1] = 230;
    }
    return parms;
}

// compute the gradient function uses central differences.
Mat computeMatXGradient(const Mat &mat, int ddepth) {
  Mat out(mat.rows,mat.cols,ddepth);

  for (int y = 0; y < mat.rows; ++y) {
    const double *Mr = mat.ptr<double>(y);
    double *Or = out.ptr<double>(y);

    Or[0] = Mr[1] - Mr[0];
    for (int x = 1; x < mat.cols - 1; ++x) {
      Or[x] = (Mr[x+1] - Mr[x-1])/2.0;
    }
    Or[mat.cols-1] = Mr[mat.cols-1] - Mr[mat.cols-2];
  }

  return out;
}

// calculates the (dx, dy) disparity of each keypoint.
Track** init_tracks(Mat I[], int &nT, Params *params){
    int _nT = 0;
    Track** T1 = loadMatches(I[0], I[1], _nT);
    Track** T = new Track*[_nT];
    for(int t=0; t<_nT; t++){
        T1[t]->dispx = abs(T1[t]->pts[params->vc].x - T1[t]->pts[1-params->vc].x);
        T1[t]->dispy = abs(T1[t]->pts[params->vc].y - T1[t]->pts[1-params->vc].y);
        if(T1[t]->dispx>params->dispRng[1]) continue;
        T[nT] = T1[t];
        nT++;
    }
    return T;
}

// initialize the normalized arrays In, Igrad and Ilab.
// we also construct the superpixels and stores their info
// in Isp and nsp.
void init_arrays(Mat I[], Mat In[], Mat Ilab[], Mat Igrad[], Mat Isp[], int nsp[], int in_nsp){
    SLIC* slice = new SLIC();
    for(int i=0; i< 2; i++){
        Mat Itmp, Igray, I4c, Itmpl;
        cvtColor(I[i], I4c, CV_BGR2BGRA);
        cvtColor(I[i], Itmpl, CV_BGR2Lab);
        Itmpl.convertTo(Ilab[i], CV_64F, 1.0/255.0);
        cvtColor(I[i], Itmp, CV_BGR2GRAY);
        I[i].convertTo(In[i], CV_64FC3, 1.0/255.0);
        Itmp.convertTo(Igray, CV_64F, 1.0/255.0);
        Igrad[i] = computeMatXGradient(Igray)+0.5;

        int *ptr;
        // calculate and show superpixels
        slice->DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels((unsigned int*)I4c.data,
                                                                   I4c.cols,
                                                                   I4c.rows,
                                                                   ptr, nsp[i],
                                                                   in_nsp, 6);
        Isp[i] = Mat(I4c.rows, I4c.cols, CV_32S, ptr);
    }
}


