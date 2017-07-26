#ifndef DEBUGGER_H
#define DEBUGGER_H

#include "../../Qing/qing_common.h"

class StereoFlow;

class Debugger
{
public:
    Debugger(const string& dir);
    ~Debugger();

    void save_init_infos(const int level);
    void save_seed_infos(const int level);
    void save_prop_infos(const int level);
    void save_rematch_infos(const int level);
    void save_upsamling_infos(const int level);
    void save_so_infos(const int level);
    void save_rv_infos(const int level);
    void save_median_infos(const int level);
    void save_subpixel_infos(const int level);
    void save_final_infos(const int level);

    void set_data_source(StereoFlow * stereo_fl);
    void get_disp_datas(Mat& disp_l, Mat& disp_r, Mat& disp);
    void save_disp_data(const string& savefn, const Mat& disp);
    void save_float_vec_data(const string& savefn, const int w, const int h, const vector<float>& vec);
    void save_int_vec_data(const string& savefn, const int w, const int h, const vector<int>& vec);

private:
    string m_save_dir;                  //save dir
    float m_scale;                      //disparity scale
    StereoFlow * m_stereo_fl;           //data sources
};


#endif // DEBUGGER_H
