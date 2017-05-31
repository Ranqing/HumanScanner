#include "cross_shaped_region.h"
#include "../../Qing/qing_image.h"

void CrossShapedRegion::set_patch_params(const int sz) {
    m_dis2 = sz;
    m_dis1 = 2 * m_dis2;
    m_tau1 = TAU1;
    m_tau2 = TAU2;

# if 0
    cout << "\tcross shaped region: dis1 = " << m_dis1 << ", dis2 = " << m_dis2 << ", tau1 = " << m_tau1 << ", tau2 = " << m_tau2 ;
# endif
}

void CrossShapedRegion::set_image_sz(const int w, const int h) {
    m_w = w;
    m_h = h;
}

void CrossShapedRegion::calc_patch_borders(const vector<float> &imgL, const vector<float> &imgR, const vector<uchar> &mskL, const vector<uchar> &mskR) {

    cout << "\n\tcalculate boarders of cross shaped region.." ;
    double duration = (double)getTickCount();
    cout << "up.." ;       calc_patch_borders(imgL, mskL,  0, -1, m_u_borders_l); calc_patch_borders(imgR, mskR, 0, -1, m_u_borders_r);
    cout << "down..";      calc_patch_borders(imgL, mskL,  0,  1, m_d_borders_l); calc_patch_borders(imgR, mskR, 0,  1, m_d_borders_r);
    cout << "left..";      calc_patch_borders(imgL, mskL, -1,  0, m_l_borders_l); calc_patch_borders(imgR, mskR, -1, 0, m_l_borders_r);
    cout << "right..";     calc_patch_borders(imgL, mskL,  1,  0, m_r_borders_l); calc_patch_borders(imgR, mskR,  1, 0, m_r_borders_r);
    duration = ((double)getTickCount() - duration) / getTickFrequency();
    printf("%.2lf s\n", duration);

# if 0
    Mat test_border(m_h, m_w, CV_32FC3);
    qing_vec_2_img<float>(imgL, test_border);
    cvtColor(test_border, test_border, CV_RGB2BGR);

    int px = 188, py = 388;
    int index = py * m_w + px;
    int uborder = m_u_borders_l[index];
    int dborder = m_d_borders_l[index];
    int lborder = m_l_borders_l[index];
    int rborder = m_r_borders_l[index];
    cout << "......test..... u_border = " << uborder << ", d_border = " << dborder << ", l_border = " << lborder << ", r_border = " << rborder << endl;

    for(int i = 0; i < uborder; i++) {
        int cx = px;
        int cy = max(py - i, 0);
        test_border.at<Vec3f>(cy, cx) = Vec3f(1.f, 0.f, 0.f);         //blue up
    }
    for(int i = 0; i < dborder; i++) {
        int cx = px;
        int cy = max(py + i, 0);
        test_border.at<Vec3f>(cy, cx) = Vec3f(0.f, 1.f, 0.f);         //green down
    }
    for(int i = 0; i < lborder; i++) {
        int cx = px - i;
        int cy = py;
        test_border.at<Vec3f>(cy, cx) = Vec3f(0.f, 0.f, 0.0f);        //black left
    }
    for(int i = 0; i < rborder; i++) {
        int cx = px + i;
        int cy = py;
        test_border.at<Vec3f>(cy, cx) = Vec3f(1.f, 0.f, 1.f);         //purple right
    }
    circle(test_border, Point2f(px, py), 2, Scalar(0.f, 0.f, 1.f));   //red center

    imshow("test_border", test_border);
    waitKey(0);
    destroyWindow("test_border");
# endif
}

void CrossShapedRegion::calc_patch_borders(const vector<float> &img, const vector<uchar> &msk, const int dx, const int dy, vector<int> &border) {
    border.resize(m_w*m_h);
    memset(&border.front(), -1, sizeof(int)*m_w*m_h);

//#pragma omp parallel for
    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int index = y * m_w + x;
            if(255 == msk[index]) {

                border[index] = calc_border(img, msk, x, y, dx, dy);
                // cout << y << ", " << x <<  ": " << border[index] << endl;
            }
        }
    }
}

int CrossShapedRegion::calc_border(const vector<float> &img, const vector<uchar> &msk, const int x, const int y, const int dx, const int dy) {
    int d = 1;
    int index = y * m_w + x;

    //center color
    float cen_r = img[index * 3 + 0];
    float cen_g = img[index * 3 + 1];
    float cen_b = img[index * 3 + 2];
    //predecerssor color
    float pre_r = cen_r, pre_g = cen_g, pre_b = cen_b;

    int xpos = x + d * dx;
    int ypos = y + d * dy;

    while(0 <= xpos && xpos < m_w && 0 <= ypos && ypos < m_h) {
        index = ypos * m_w + xpos;

        float cur_r = img[index * 3 + 0];
        float cur_g = img[index * 3 + 1];
        float cur_b = img[index * 3 + 2];

        float delta_r_1 = cur_r - cen_r; float delta_g_1 = cur_g - cen_g; float delta_b_1 = cur_b - cen_b;
        float delta_r_2 = cur_r - pre_r; float delta_g_2 = cur_g - pre_g; float delta_b_2 = cur_b - pre_b;

        //color differene between cur_pixel and cen_pixel
        float color_diff_1 = sqrt(delta_r_1 * delta_r_1 + delta_g_1 * delta_g_1 + delta_b_1 * delta_b_1) * 0.3;
        //color difference between cur_pixel and pre_pixel
        float color_diff_2 = sqrt(delta_r_2 * delta_r_2 + delta_g_2 * delta_g_2 + delta_b_2 * delta_b_2) * 0.3;

        bool condition1 = msk[index];
        bool condition2 = d < m_dis1;
        bool condition3 = ( color_diff_1 < m_tau1 ) && ( color_diff_2 < m_tau1 );      //difference with center pixel and previous pixel small than m_tau1
        bool condition4 = d < m_dis2 || ( d > m_dis2 && color_diff_1 < m_tau2 );

        if( !(condition1 && condition2 && condition3 && condition4)) break;

        ++d;
        pre_r = cur_r, pre_g = cur_g, pre_b = cur_b;
        xpos += dx;
        ypos += dy;
    }
    return d - 1;
}
