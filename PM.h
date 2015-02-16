///* Minimal (unoptimized) example of PatchMatch. Requires that ImageMagick be installed.

//  To improve generality you can:
//   - Use whichever distance function you want in dist(), e.g. compare SIFT descriptors computed densely.
//   - Search over a larger search space, such as rotating+scaling patches (see MATLAB mex for examples of both)

//  To improve speed you can:
//   - Turn on optimizations (/Ox /Oi /Oy /fp:fast or -O6 -s -ffast-math -fomit-frame-pointer -fstrength-reduce -msse2 -funroll-loops)
//   - Use the MATLAB mex which is already tuned for speed
//   - Use multiple cores, tiling the input. See our publication "The Generalized PatchMatch Correspondence Algorithm"
//   - Tune the distance computation: manually unroll loops for each patch size, use SSE instructions (see readme)
//   - Precompute random search samples (to avoid using rand, and mod)
//   - Move to the GPU
//  -------------------------------------------------------------------------- */


//#ifndef PM_H
//#define PM_H
//#endif // PM_H

//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <limits.h>
//#include <math.h>
//#include <Camera.h>
//#include <kdtree.h>
//#include <GuidedFilter.h>

//typedef unsigned char uchar;
//typedef unsigned short ushort;

//#ifndef MAX
//#define MAX(a, b) ((a)>(b)?(a):(b))
//#define MIN(a, b) ((a)<(b)?(a):(b))
//#endif

///* -------------------------------------------------------------------------
//   PatchMatch, using L2 distance between upright patches that translate only
//   ------------------------------------------------------------------------- */

//#define XY_TO_INT(x, y) (((y)<<12)|(x))
//#define INT_TO_X(v) ((v)&((1<<12)-1))
//#define INT_TO_Y(v) ((v)>>12)

//struct PMRect {
//  int minx = INT_MAX, miny = INT_MAX, maxx = 0, maxy = 0;
//  int pminx = INT_MAX, pminy = INT_MAX, pmaxx = 0, pmaxy = 0; // padded
//  int *es;
//  int nes = 0;
//  int *knn; // points to th most similar neighbors (Max K)
//  double *knnd;
//  double *cts; // paiwise costs
//  int nk = 0;

//  int mr = 0;
//  int mg = 0;
//  int mb = 0;
//  int npix = 0;
//  int id = -1;
//} ;


///* -------------------------------------------------------------------------
//   PMImage: Minimal image class
//   ------------------------------------------------------------------------- */
//template <class T>
//class PMImage {
//    public:
//        int w, h;
//        T *data;

//        PMImage<T>(int w_, int h_) :w(w_), h(h_), create_data(true){data = new T[w*h];}

//        PMImage<T>(T *_data, int w_, int h_, bool _create_data=false) :w(w_), h(h_), create_data(_create_data){
//            if(_create_data){
//                data = new T[w*h];
//                memcpy(data, _data, w*h*sizeof(T));
//            }else{
//                data=_data;
//            }
//        }
//        ~PMImage(){if(create_data) delete[] data;}

//        T* operator[](int y) {return &data[y*w]; }
//    private:
//        bool create_data = false;
//};

//class PM{
//    public:
//        int patch_w;
//        int pm_iters;
//        int K = 7;
//        double rs_max;
//        double Lambda1 = 5;
//        double Lambda2 = 20;
//        double Wc;
//        double Beta = 0.5;
//        double Gamma1 = 3;
//        double Gamma2 = 4;
//        double eps = pow(0.1,2.0);

//        PM(int _patch_w=7, int _pm_iters=5):patch_w(_patch_w), pm_iters(_pm_iters){rs_max = 10*patch_w; Wc=4
//                                                                                  *patch_w;}
//        ~PM();
//        void patchmatch(PMImage<int> **_fs, PMImage<double> **fs_grad, PMImage<double> **fs_gray, int _n_fs, int vc, int **_rgs, int *_n_rgs, double *range, Camera **cams,
//                        Track **tracks, int _n_tracks, double *out_lbs, Mat *test_a, Mat *test_b);

//    private:
//        void improve_guess(PMRect *a_rec, PMImage<int> *a, PMImage<double> *a_gray,
//                           PMImage<double> *a_grad, PMImage<int> *b, PMImage<double> *b_grad,
//                           Mat Pab, double d, PMImage<double> *a_lbs,
//                           PMImage<int> *a_rgs, PMImage<double> *a_cts,
//                           PMImage<Point2d> *a_nn);

//        double dist(PMImage<int> *, PMImage<int> *,
//                     PMImage<double> *, PMImage<double> *,
//                    int, int, int, int);
//        double dist(PMRect *rec1, PMRect *rec2);
//        double dist(int a, int b, double ag, double bg);
//        double pairwise(PMRect *, PMRect *, int, int);
//        double unary(PMRect *, int);
//        double propagate(PMImage<int> *b_rgs, PMRect *a_recs,
//                         PMRect *b_recs, Mat P, double d,
//                         int a_id);
//        void rgb2lab(int rgb, int & lab);
//        void rgb2xyz(const int&, const int&, const int&, double& , double&, double&);
//        void conv_rgb2lab(PMImage<int> *&, PMImage<int> *&);
//        void gen_recs(PMImage<int> *, PMImage<int> *, int , PMRect *, int);
//        void gen_graph(PMImage<int> *, int , PMRect *);
//        kdtree *gen_tracks_tree(Track **tracks, int n_tracks, int pidx);
//        double norm(int, int);
//};
