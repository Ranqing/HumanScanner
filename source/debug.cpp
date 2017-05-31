#include "debug.h"
#include "stereo_flow.h"

#include "../../Qing/qing_image.h"

Debugger::Debugger(const string& dir):m_save_dir(dir)  {
}

Debugger::~Debugger() {

}

void Debugger::set_data_source(StereoFlow *stereo_fl) {
    m_stereo_fl = stereo_fl;
    m_scale = m_stereo_fl->get_scale();
}

void Debugger::get_disp_datas(Mat& disp_l, Mat& disp_r, Mat& disp) {
    vector<float>& disp_vec_l = m_stereo_fl->get_disp_l();
    vector<float>& disp_vec_r = m_stereo_fl->get_disp_r();
    vector<float>& disp_vec = m_stereo_fl->get_disp();

    int w = m_stereo_fl->get_w();
    int h = m_stereo_fl->get_h();

    disp_l = Mat::zeros(h, w, CV_32FC1);
    disp_r = Mat::zeros(h, w, CV_32FC1);
    disp = Mat::zeros(h, w, CV_32FC1);

    qing_vec_2_img<float>(disp_vec_l, disp_l);
    qing_vec_2_img<float>(disp_vec_r, disp_r);
    qing_vec_2_img<float>(disp_vec, disp);
}

void Debugger::save_disp_data(const string &savefn, const Mat &disp) {
    Mat disp_img;
    disp.convertTo(disp_img, CV_8UC1, m_scale);
    imwrite(savefn, disp_img);
}

void Debugger::save_float_vec_data(const string& savefn, const int w, const int h, const vector<float>& vec){
    ofstream fout(savefn, ios::out);
    if(fout.is_open() == true) {
        for(int x = 0, cnt = 0; x < vec.size(); ++x) {
            fout << setprecision(4) << vec[x] << ' ';
            if(cnt == w)  {
                fout << endl;
                cnt = 0;
            }
            else cnt++;
        }
    }
    fout.close();

}

void Debugger::save_int_vec_data(const string &savefn, const int w, const int h, const vector<int> &vec) {
    ofstream fout(savefn, ios::out);
    if(fout.is_open() == true) {
        for(int x = 0, cnt = 0; x < vec.size(); ++x) {
            fout <<  vec[x] << ' ';
            if(cnt == w)  {
                fout << endl;
                cnt = 0;
            }
            else cnt++;
        }
    }
    fout.close();
}

void Debugger::save_init_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/init_disp_l_" + str + ".jpg";
    string save_fn_r = m_save_dir + "/init_disp_r_" + str + ".jpg";
    string save_fn   = m_save_dir + "/init_disp_" + str + ".jpg";
    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;

# if 0
    int ranges = m_stereo_fl->get_disp_range();
    int w = m_stereo_fl->get_w();
    int h = m_stereo_fl->get_h();
    save_fn_l = m_save_dir + "/best_mcost_l_" + str + ".txt";
    save_float_vec_data(save_fn_l, w, h, m_stereo_fl->get_best_mcost_l());
    save_fn_r = m_save_dir + "/best_mcost_r_" + str + ".txt";
    save_float_vec_data(save_fn_r, w, h, m_stereo_fl->get_best_mcost_r());
    cout << "debug:\tsaving " << save_fn_l  << '\t' << save_fn_r << endl;
    save_fn_l = m_save_dir + "/best_prior_l_" + str + ".txt";
    save_float_vec_data(save_fn_l, w, h, m_stereo_fl->get_best_prior_l());
    save_fn_r = m_save_dir + "/best_prior_r_" + str + ".txt";
    save_float_vec_data(save_fn_r, w, h, m_stereo_fl->get_best_prior_r());
    cout << "debug:\tsaving " << save_fn_l  << '\t' << save_fn_r << endl;
    save_fn_l = m_save_dir + "/best_disp_l_" + str + ".txt";
    save_float_vec_data(save_fn_l, w, h, m_stereo_fl->get_disp_l());
    save_fn_r = m_save_dir + "/best_disp_r_" + str + ".txt";
    save_float_vec_data(save_fn_r, w, h, m_stereo_fl->get_disp_r());
    cout << "debug:\tsaving " << save_fn_l  << '\t' << save_fn_r << endl;

    save_fn_l = m_save_dir + "/best_k_l_" + str + ".txt";
    save_int_vec_data(save_fn_l, w, h, m_stereo_fl->get_bestk_l());
    save_fn_r = m_save_dir + "/best_k_r_" + str + ".txt";
    save_int_vec_data(save_fn_r, w, h, m_stereo_fl->get_bestk_r());
     cout << "debug:\tsaving " << save_fn_l  << '\t' << save_fn_r << endl;

//    for(int i = 0; i < ranges; ++i) {
//        save_fn_l = m_save_dir + "/mcost_l_" + str + "_" + int2string(i) + ".txt";
//        save_float_vec_data(save_fn_l, w, h, m_stereo_fl->get_mcost_l(i));
//        save_fn_r = m_save_dir + "/mcost_r_" + str + "_" + int2string(i) + ".txt";
//        save_float_vec_data(save_fn_r, w, h, m_stereo_fl->get_mcost_r(i));
//        cout << "debug:\tsaving " << save_fn_l << "\t" << save_fn_r << endl;
//    }
# endif
}

void Debugger::save_seed_infos(const int level) {
    int w = m_stereo_fl->get_w();
    int h = m_stereo_fl->get_h();

    Mat disp_seed = Mat::zeros(h, w, CV_32FC1);
    vector<float>& disp_vec_seed = m_stereo_fl->get_disp_seed();
    qing_vec_2_img<float>(disp_vec_seed, disp_seed);

    string save_fn = m_save_dir + "/seed_disp_" + int2string(level) + ".jpg";
    save_disp_data(save_fn, disp_seed);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_prop_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/prop_disp_l_" + str + ".jpg";
    string save_fn_r = m_save_dir + "/prop_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/prop_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_rematch_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/fill_disp_l_" + str + ".jpg";           //i.e. rematch
    string save_fn_r = m_save_dir + "/fill_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/fill_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_upsamling_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/upsamling_disp_l_" + str + ".jpg";
    string save_fn_r = m_save_dir + "/upsamling_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/upsamling_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_so_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/so_disp_l_" + str + ".jpg";
    string save_fn_r = m_save_dir + "/so_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/so_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_rv_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/rv_disp_l_" + str + ".jpg";
    string save_fn_r = m_save_dir + "/rv_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/rv_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_median_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/mf_disp_l_" + str + ".jpg";          //i.e. median filter
    string save_fn_r = m_save_dir + "/mf_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/mf_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);
    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);
    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_final_infos(const int level) {
    Mat disp_l, disp_r, disp;
    get_disp_datas(disp_l, disp_r, disp);

    string str = int2string(level);
    string save_fn_l = m_save_dir + "/final_disp_l_" + str + ".jpg";
    string save_fn_r = m_save_dir + "/final_disp_r_"  + str + ".jpg";
    string save_fn   = m_save_dir + "/final_disp_" + str + ".jpg";

    save_disp_data(save_fn_l, disp_l);    cout << "debug:\tsaving " << save_fn_l << endl;
    save_disp_data(save_fn_r, disp_r);    cout << "debug:\tsaving " << save_fn_r << endl;
    save_disp_data(save_fn, disp);    cout << "debug:\tsaving " << save_fn << endl;

    int w = m_stereo_fl->get_w();
    int h = m_stereo_fl->get_h();
    vector<float>& disp_vec_l = m_stereo_fl->get_disp_l();
    vector<float>& disp_vec_r = m_stereo_fl->get_disp_r();
    vector<float>& disp_vec = m_stereo_fl->get_disp();

    save_fn_l = m_save_dir + "/final_disp_l_" + str + ".txt"; save_float_vec_data(save_fn_l, w, h, disp_vec_l); cout << "debug:\tsaving " << save_fn_l << endl;
    save_fn_r = m_save_dir + "/final_disp_r_" + str + ".txt"; save_float_vec_data(save_fn_r, w, h, disp_vec_r); cout << "debug:\tsaving " << save_fn_r << endl;
    save_fn = m_save_dir + "/final_disp_" + str + ".txt"; save_float_vec_data(save_fn, w, h, disp_vec); cout << "debug:\tsaving " << save_fn << endl;

    Mat mask_l = m_stereo_fl->get_mat_mask_l();
    Mat mask_r = m_stereo_fl->get_mat_mask_r();
    int wnd_sz = m_stereo_fl->get_wnd_size();
    Mat erode_mask_l =  qing_erode_image(mask_l, 2*wnd_sz);
    Mat erode_mask_r =  qing_erode_image(mask_r, 2*wnd_sz);
    Mat erode_disp_l, erode_disp_r, erode_disp;
    disp_l.copyTo(erode_disp_l, erode_mask_l);
    disp_r.copyTo(erode_disp_r, erode_mask_r);
    disp.copyTo(erode_disp, erode_mask_l);

    save_fn_l = m_save_dir + "/erode_final_disp_l_" + str + ".jpg"; save_disp_data(save_fn_l, erode_disp_l);    cout << "debug:\tsaving " << save_fn_l << endl;
    save_fn_r = m_save_dir + "/erode_final_disp_r_" + str + ".jpg"; save_disp_data(save_fn_r, erode_disp_r);    cout << "debug:\tsaving " << save_fn_r << endl;
    save_fn = m_save_dir + "/erode_final_disp_" + str + ".jpg"; save_disp_data(save_fn, erode_disp);    cout << "debug:\tsaving " << save_fn << endl;

    int total = w * h;
    vector<float> erode_disp_vec_l(total);
    vector<float> erode_disp_vec_r(total);
    vector<float> erode_disp_vec(total);
    memcpy(&erode_disp_vec_l.front(), erode_disp_l.data, sizeof(float)*total );
    memcpy(&erode_disp_vec_r.front(), erode_disp_r.data, sizeof(float)*total );
    memcpy(&erode_disp_vec.front(), erode_disp.data, sizeof(float)*total);

    save_fn_l = m_save_dir + "/erode_final_disp_l_" + str + ".txt"; save_float_vec_data(save_fn_l, w, h, erode_disp_vec_l); cout << "debug:\tsaving " << save_fn_l << endl;
    save_fn_r = m_save_dir + "/erode_final_disp_r_" + str + ".txt"; save_float_vec_data(save_fn_r, w, h, erode_disp_vec_r); cout << "debug:\tsaving " << save_fn_r << endl;
    save_fn = m_save_dir + "/erode_final_disp_" + str + ".txt"; save_float_vec_data(save_fn, w, h, erode_disp_vec); cout << "debug:\tsaving " << save_fn << endl;
}

void Debugger::save_subpixel_infos(const int level) {

    string str = int2string(level);
    string save_fn = m_save_dir + "/subpixel_final_disp_" + str + ".txt";

    int w = m_stereo_fl->get_w();
    int h = m_stereo_fl->get_h();
    vector<float>& disp = m_stereo_fl->get_disp();
    save_float_vec_data(save_fn, w, h, disp);
    cout << "debug:\tsaving " << save_fn << endl;
}
