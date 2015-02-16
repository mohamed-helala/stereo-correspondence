#ifndef PARAMS_H
#define PARAMS_H

#endif // PARAMS_H

enum Calculation{
    Calc_PatchMatch,
    Calc_VolumeFiltering,
    Calc_PatchMatch_Salient,
    Calc_VolumeFiltering_Salient
};

class Params {
    public:
    double dispRng[2];
    bool show_stages, debug;
    bool all_results, main_results;
    bool intermed_yaml, read_yaml;
    char *suffix;
    int nsp; // number of superpixels;
    double occlThr;
    int n_int_planes; // number of intermediate planes in case of complex transformation
    double depthstep;
    int scale;
    double rs;
    int uw;
    double eps;
    double r;
    int vc; // virtual camera
    double consThr[2]; // consistency threshold
    char* indirectory;
    char* left_image;
    char* left_binary;
    char* right_binary;
    char* leftg_binary;
    char* rightg_binary;
    char* right_image;
    char* cams_file;
    char* outdirectory;
    char* input_image;
    char* input_disp;
    Calculation calc;
    Params():show_stages(false), debug(false),
        intermed_yaml(false), read_yaml(false),
        occlThr(0.3), scale(1), rs(0.4),
        eps(pow(0.01,2.0)), r(9){}
};
