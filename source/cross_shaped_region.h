#ifndef SUPPORT_H
#define SUPPORT_H

#include <iostream>
#include <vector>
using namespace std;

#include <opencv2/core/core.hpp>
using namespace cv;

#define TAU1 20
#define TAU2 6

//class HumanBodyScanner;

class CrossShapedRegion
{
public:
    CrossShapedRegion();
    CrossShapedRegion(const int tau1, const int tau2, const int dis1, const int dis2);
    ~CrossShapedRegion();

    void set_patch_params(const int sz);
    void set_image_sz(const int w, const int h);
    void calc_patch_borders(const vector<float>& imgL, const vector<float>& imgR, const vector<uchar>& mskL, const vector<uchar>& mskR);
    void calc_patch_borders(const vector<float>& img, const vector<uchar>& msk, const int dx, const int dy, vector<int>& border);

    int calc_border(const vector<float>& img, const vector<uchar>& msk, const int x, const int y, const int dx, const int dy);

private:
    //params in paper of cross shaped region
    int m_tau1, m_tau2, m_dis1, m_dis2;
    //image size
    int m_w, m_h;

    //HumanBodyScanner& m_scanner;                                //using original class data

    //cross shaped region
    vector<int> m_u_borders_l, m_d_borders_l, m_l_borders_l, m_r_borders_l;
    vector<int> m_u_boarder_r, m_d_borders_r, m_l_borders_r, m_r_borders_r;
};


inline CrossShapedRegion::CrossShapedRegion(const int tau1, const int tau2, const int dis1, const int dis2):
    m_tau1(tau1), m_tau2(tau2), m_dis1(dis1), m_dis2(dis2) {
}

inline CrossShapedRegion::CrossShapedRegion() {


}

inline CrossShapedRegion::~CrossShapedRegion() {

}




#endif // SUPPORT_H
