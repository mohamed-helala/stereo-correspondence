/* Minimal (unoptimized) example of PatchMatch. Requires that ImageMagick be installed.

  To improve generality you can:
   - Use whichever distance function you want in dist(), e.g. compare SIFT descriptors computed densely.
   - Search over a larger search space, such as rotating+scaling patches (see MATLAB mex for examples of both)

  To improve speed you can:
   - Turn on optimizations (/Ox /Oi /Oy /fp:fast or -O6 -s -ffast-math -fomit-frame-pointer -fstrength-reduce -msse2 -funroll-loops)
   - Use the MATLAB mex which is already tuned for speed
   - Use multiple cores, tiling the input. See our publication "The Generalized PatchMatch Correspondence Algorithm"
   - Tune the distance computation: manually unroll loops for each patch size, use SSE instructions (see readme)
   - Precompute random search samples (to avoid using rand, and mod)
   - Move to the GPU
  -------------------------------------------------------------------------- */


#ifndef PM_H
#define PM_H
#endif // PM_H

#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <cmath>
#ifndef CAMER_H
#include <Camera.h>
#endif

#include <kdtree.h>
#include <GuidedFilter.h>

typedef unsigned char uchar;
typedef unsigned short ushort;

#ifndef MAX
#define MAX(a, b) ((a)>(b)?(a):(b))
#define MIN(a, b) ((a)<(b)?(a):(b))
#endif

/* -------------------------------------------------------------------------
   PatchMatch, using L2 distance between upright patches that translate only
   ------------------------------------------------------------------------- */

#define XY_TO_INT(x, y) (((y)<<12)|(x))
#define INT_TO_X(v) ((v)&((1<<12)-1))
#define INT_TO_Y(v) ((v)>>12)

struct PMRect {
  int minx = INT_MAX, miny = INT_MAX, maxx = 0, maxy = 0;
  int pminx = INT_MAX, pminy = INT_MAX, pmaxx = 0, pmaxy = 0; // padded
  int *es;
  int nes = 0;
  int *knn; // points to th most similar neighbors (Max K)
  double *knnd;
  double *cts; // paiwise costs
  int nk = 0;

  int mr = 0;
  int mg = 0;
  int mb = 0;
  int npix = 0;
  int id = -1;
} ;


/* -------------------------------------------------------------------------
   PMImage: Minimal image class
   ------------------------------------------------------------------------- */
template <class T>
class PMImage {
    public:
        int w, h;
        T *data;

        PMImage<T>(int w_, int h_) :w(w_), h(h_), create_data(true){data = new T[w*h];}

        PMImage<T>(T *_data, int w_, int h_, bool _create_data=false) :w(w_), h(h_), create_data(_create_data){
            if(_create_data){
                data = new T[w*h];
                memcpy(data, _data, w*h*sizeof(T));
            }else{
                data=_data;
            }
        }
        ~PMImage(){if(create_data) delete[] data;}

        T* operator[](int y) {return &data[y*w]; }
    private:
        bool create_data = false;
};

class PM{
    public:
        int patch_w;
        int pm_iters;
        int K = 14;
        double rs_max;
        bool test = false;
        int r;
        int pad;
        double Lambda = 5;
        double *Rng;
        double Wc;
        double GAMMA = 0.11;
        double THCOLOR = 7.0/255.0;
        double THGRAD = 2.0/255.0;
        double eps = pow(0.1,2.0);

        PM(int _patch_w=7, int _pm_iters=5):patch_w(_patch_w), pm_iters(_pm_iters){rs_max = 10*patch_w; Wc=4
                                                                                  *patch_w;}
        ~PM();
        void patchmatch(Mat *I, Mat *Igrad, int nI, int vc, Mat *Isp, int *nsp, Camera **C,
                        Track **T, int nT, Mat &disp, Mat &a_test, Mat &b_test);

        void gen_recs(Mat I, Mat Isp, int nsp, PMRect *Rs, int pad);
        void gen_graph(Mat Isp, int nsp, PMRect* Rc);
    private:
        void improve_guess(PMRect *a_rec, Mat a, Mat a_grad,
                           Mat a_sp, Mat b, Mat b_grad,
                           Mat Pab, double d, Mat *disp, Mat *cost, Mat &a_test, Mat &b_test);

        double dist(Mat a, Mat b,
                    Mat a_grad, Mat b_grad,
                    int xa, int ya, int xb, int yb);
        double dist(PMRect *rec1, PMRect *rec2);
        double dist(Vec3d a, Vec3d b, double ag, double bg);
        Mat calc_dist(PMRect *b_rec, PMRect *b_in, Mat b, Mat b_grad, Mat Wa, Mat Wa_grad, bool filter= false);
        double propagate(Mat b_sp, PMRect *a_Rc,
                          PMRect *b_Rc, Mat P, double d,
                          int a_id);

        kdtree *gen_tracks_tree(Track **tracks, int n_tracks, int pidx);
        double norm(Vec3d ac, Vec3d bc);
        PMRect *proj_sp(PMRect *a_rec, Mat a, Mat a_grad, Mat b, Mat Pab, Mat &Wa, Mat &Wa_grad);
};
