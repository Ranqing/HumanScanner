#include "stereo_flow.h"
#include "cost/zncc.h"
#include "cost/census.h"
#include "cost/tad.h"
#include "aggr/bf_aggr.h"
#include "aggr/box_aggr.h"
#include "qx_upsampling/qx_zdepth_upsampling_using_rbf.h"
#include "hashmap.h"


#include "../../Qing/qing_timer.h"
#include "../../Qing/qing_memory.h"
#include "../../Qing/qing_median_filter.h"
#include "../../Qing/qing_matching_cost.h"


const CCType CCNAME = zncc;
const CAType CANAME = bf;

void StereoFlow::set_patch_params(const int sz) {
    m_support_region->set_image_sz(m_w, m_h);
    m_support_region->set_patch_params(sz);
}

CostMethod * StereoFlow::set_cost_type(const CCType name) {
    switch(name) {
    case cen: return new Census(m_w, m_h, m_disp_ranges, m_max_disp, m_min_disp, DEFAULT_CEN_WND); break;     //wnd_sz = 0
    case zncc: return new ZNCC(m_w, m_h, m_disp_ranges, m_max_disp, m_min_disp, DEFAULT_NCC_WND);  break;
    case tad: return new TAD(m_w, m_h, m_disp_ranges, m_max_disp, m_min_disp); break;
    default: return NULL;
    }
    return NULL;
}

AggrMethod * StereoFlow::set_aggr_type(const CAType name) {
    switch(name) {
    case bf: return new BFCA(m_w,m_h, m_disp_ranges, m_wnd_size); break;
    case box: return new BoxCA(m_w, m_h, m_disp_ranges, m_wnd_size); break;
    default: return NULL;
    }
    return NULL;
}

void StereoFlow::calc_mean_images() {

    //unsigned char [0,255]
    cv::blur(m_mat_gray_l, m_mat_mean_l, Size(m_wnd_size, m_wnd_size));
    cv::blur(m_mat_gray_r, m_mat_mean_r, Size(m_wnd_size, m_wnd_size));

    //float [0,255]
    Mat meanL, meanR;
    m_mat_mean_l.convertTo(meanL, CV_32FC1); qing_img_2_vec<float>(meanL, m_mean_l);             //imshow("mean_l", m_mat_mean_l); waitKey(0); destroyWindow("mean_l");
    m_mat_mean_r.convertTo(meanR, CV_32FC1); qing_img_2_vec<float>(meanR, m_mean_r);             //imshow("mean_r", m_mat_mean_r); waitKey(0); destroyWindow("mean_r");

# if 0
 //   m_ncc_mean_l.resize(m_total); copy(m_mean_l.begin(), m_mean_l.end(), m_ncc_mean_l.begin());
 //   m_ncc_mean_r.resize(m_total); copy(m_mean_r.begin(), m_mean_r.end(), m_ncc_mean_r.begin());
 //   cout << "\tmean image with default_ncc_wnd done...." << endl;
# endif
}

void StereoFlow::calc_support_region() {
    m_support_region->calc_patch_borders(m_view_l, m_view_r, m_mask_l, m_mask_r );
}

void StereoFlow::calc_cost_vol() {
    if(m_cost_mtd) {
# if 1
      //  cout << "\tusing ncc mean image...." << endl;
      //  m_cost_mtd->build_cost_vol_l(m_gray_l, m_gray_r, m_ncc_mean_l, m_ncc_mean_r, m_mask_l, m_mask_r, m_cost_vol_l);
      //  m_cost_mtd->build_cost_vol_r(m_gray_l, m_gray_r, m_ncc_mean_l, m_ncc_mean_r, m_mask_l, m_mask_r, m_cost_vol_r);
        m_cost_mtd->build_cost_vol_l(m_gray_l, m_gray_r, m_mean_l, m_mean_r, m_mask_l, m_mask_r, m_cost_vol_l);
        m_cost_mtd->build_cost_vol_r(m_gray_l, m_gray_r, m_mean_l, m_mean_r, m_mask_l, m_mask_r, m_cost_vol_r);
#else
        m_cost_mtd->build_cost_vol_l(m_gray_l, m_gray_r, m_mean_l, m_mean_r, m_mask_l, m_mask_r, m_cost_vol_l);
        m_cost_mtd->build_cost_vol_r(m_gray_l, m_gray_r, m_mean_l, m_mean_r, m_mask_l, m_mask_r, m_cost_vol_r);
# endif
    }
    else {
        cerr << "no cost method is claimed, do nothing..." << endl;
    }
}

void StereoFlow::aggr_cost_vol() {
    if(m_aggr_mtd) {
        m_aggr_mtd->aggr_cost_vol(m_gray_l, m_mask_l, m_cost_vol_l);
        m_aggr_mtd->aggr_cost_vol(m_gray_r, m_mask_r, m_cost_vol_r);
    }
    else {
        cerr << "no aggr method is claimed, do nothing..." << endl;
    }
}

void StereoFlow::set_disp_by_wta() {
    switch(CCNAME) {
    case zncc: {
        cout << "\tmax takes all setting disparity.." << endl;
        max_takes_all(m_mask_l, m_cost_vol_l, m_disp_l, m_best_k_l, m_best_mcost_l, m_best_prior_l);
        max_takes_all(m_mask_r, m_cost_vol_r, m_disp_r, m_best_k_r, m_best_mcost_r, m_best_prior_r);
        break; }
    default: {
        cout << "\tmin takes all setting disparity.." << endl;
        min_takes_all(m_mask_l, m_cost_vol_l, m_disp_l, m_best_k_l, m_best_mcost_l, m_best_prior_l);
        min_takes_all(m_mask_r, m_cost_vol_r, m_disp_r, m_best_k_r, m_best_mcost_r, m_best_prior_r);
        break;
    }
    }
}

void StereoFlow::min_takes_all(const vector<uchar>& mask, const vector<vector<float> >& cost_vol,
                               vector<float>& disp, vector<int>& bestk, vector<float>& mcost, vector<float>& prior) {

    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx = y * m_w + x;
            if(0==mask[idx]) continue;
            int mink = 0;
            float min_mcost = cost_vol[mink][idx];
            float sec_mcost = numeric_limits<float>::max();
            for(int k = 1; k <= m_disp_ranges; ++k)
            {
                float cur_mcost = cost_vol[k][idx];
                if(cur_mcost < min_mcost) {
                    sec_mcost = min_mcost;
                    min_mcost = cur_mcost;
                    mink = k;
                }
                else if(cur_mcost < sec_mcost) {
                    sec_mcost = cur_mcost;
                }
            }

            disp[idx] = qing_k_2_disp(m_max_disp, m_min_disp, mink);
            bestk[idx] = mink;
            mcost[idx] = min_mcost;
            prior[idx] = calc_min_prior(min_mcost, sec_mcost);
        }
    }
}

void StereoFlow::max_takes_all(const vector<uchar> &mask, const vector<vector<float> > &cost_vol,
                               vector<float> &disp, vector<int> &bestk, vector<float> &mcost, vector<float> &prior) {
    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx = y * m_w + x;
            if(0==mask[idx]) continue;
            int maxk = 0;
            float max_mcost = cost_vol[maxk][idx];
            float sec_mcost = -1;

            for(int k = 1; k <= m_disp_ranges; ++k) {
                float cur_mcost = cost_vol[k][idx];
                if(cur_mcost > max_mcost) {
                    sec_mcost = max_mcost;
                    max_mcost = cur_mcost;
                    maxk = k;
                }
                else if(cur_mcost > sec_mcost) {
                    sec_mcost = cur_mcost;
                }
            }

            disp[idx] = qing_k_2_disp(m_max_disp, m_min_disp, maxk);
            bestk[idx] = maxk;
            mcost[idx] = max_mcost;
            prior[idx] = calc_max_prior(max_mcost, sec_mcost);
        }
    }
}

void StereoFlow::calc_init_disparity() {                                                    //calculate initial disparity
    m_disp_l.resize(m_total);   memset(&m_disp_l.front(), 0, sizeof(float) * m_total);
    m_disp_r.resize(m_total);   memset(&m_disp_r.front(), 0, sizeof(float) * m_total);
    cout << "\n\tcalculate initial disparity , " << m_min_disp << "~" << m_max_disp << ", disp range = " << m_disp_ranges << endl;

    m_cost_vol_l.resize(m_disp_ranges + 1);
    m_cost_vol_r.resize(m_disp_ranges + 1);

    for(int k = 0; k <= m_disp_ranges; ++k) {
        m_cost_vol_l[k].resize(m_total); memset(&m_cost_vol_l[k].front(), 0.f, sizeof(float)*m_total);
        m_cost_vol_r[k].resize(m_total); memset(&m_cost_vol_r[k].front(), 0.f, sizeof(float)*m_total);
    }

    m_cost_mtd = set_cost_type(CCNAME);
    m_aggr_mtd = set_aggr_type(CANAME);

    calc_cost_vol();
    aggr_cost_vol();

    cout << "\tcost volume calculation done..." << endl;
    set_disp_by_wta();
}

void StereoFlow::calc_cluster_costs_l(const int st_k, const int ed_k, const int lx, const int ly, vector<float> &mcosts) {
    int ncc_wnd_sz = m_wnd_size;
    int offset = ncc_wnd_sz/2;

    float mean_l = m_mean_l[ly * m_w + lx];
    vector<float> ncc_vec_l(ncc_wnd_sz * ncc_wnd_sz, 0.f);
    for(int dy = -offset; dy <= offset; ++dy) {
        int cur_y = ly + dy;
        if(0 > cur_y || m_h <= cur_y) continue;
        for(int dx = -offset; dx <= offset; ++dx) {
            int cur_x = lx + dx;
            if(0 > cur_x || m_w <= cur_x) continue;
            int cur_idx = cur_y * m_w + cur_x;
            if(0==m_mask_l[cur_idx]) continue;
            ncc_vec_l[(dy+offset)*ncc_wnd_sz + (dx+offset)] = m_gray_l[cur_idx] - mean_l;
        }
    }

    for(int k = st_k; k <= ed_k; ++k) {
        float d = qing_k_2_disp(m_max_disp, m_min_disp, k);
        if(m_min_disp > d || m_max_disp < d) continue;

        int ry = ly;
        int rx = lx - d;
        int r_cen_idx = ry * m_w + rx;

        if(0 > rx || 0 == m_mask_r[r_cen_idx]) continue;

        float mean_r = m_mean_r[r_cen_idx];
        vector<float> ncc_vec_r(ncc_wnd_sz * ncc_wnd_sz, 0.f);
        for(int dy = -offset; dy <= offset; ++dy) {
            int cur_y = ry + dy;
            if(0 > cur_y || m_h <= cur_y) continue;
            for(int dx = -offset; dx <= offset; ++dx) {
                int cur_x = rx + dx;
                if(0 > cur_x || m_w <= cur_x) continue;
                int cur_idx = cur_y * m_w + cur_x;
                if(0==m_mask_r[cur_idx]) continue;
                ncc_vec_r[(dy+offset)*ncc_wnd_sz + (dx+offset)] = m_gray_r[cur_idx] - mean_r;
            }
        }

        mcosts[k-st_k] = qing_ncc_value(ncc_vec_l, ncc_vec_r);
    }
}

void StereoFlow::calc_cluster_costs_r(const int st_k, const int ed_k, const int rx, const int ry,  vector<float> &mcosts) {
    int ncc_wnd_sz = m_wnd_size;
    int offset = ncc_wnd_sz/2;

    float mean_r = m_mean_r[ry * m_w + rx];
    vector<float> ncc_vec_r(ncc_wnd_sz * ncc_wnd_sz, 0.f);
    for(int dy = -offset; dy <= offset; ++dy) {
        int cur_y = ry + dy;
        if(0 > cur_y || m_h <= cur_y) continue;
        for(int dx = -offset; dx <= offset; ++dx) {
            int cur_x = rx + dx;
            if(0 > cur_x || m_w <= cur_x) continue;
            int cur_idx = cur_y * m_w + cur_x;
            if(0==m_mask_r[cur_idx]) continue;
            ncc_vec_r[(dy+offset)*ncc_wnd_sz + (dx+offset)] = m_gray_r[cur_idx] - mean_r;
        }
    }

    for(int k = st_k; k <= ed_k; ++k) {
        float d = qing_k_2_disp(m_max_disp, m_min_disp, k);
        if(m_min_disp > d || m_max_disp < d) continue;

        int ly = ry;
        int lx = rx + d;
        int l_cen_idx = ly * m_w + lx;

        if(m_w <= lx || 0 == m_mask_l[l_cen_idx]) continue;

        float mean_l = m_mean_l[l_cen_idx];
        vector<float> ncc_vec_l(ncc_wnd_sz * ncc_wnd_sz, 0.f);
        for(int dy = -offset; dy <= offset; ++dy) {
            int cur_y = ly + dy;
            if( 0 > cur_y || m_h <= cur_y) continue;
            for(int dx = -offset; dx <= offset; ++dx) {
                int cur_x = lx + dx;
                if(0 > cur_x || m_w <= cur_x) continue;
                int cur_idx = cur_y * m_w + cur_x;
                if(0==m_mask_l[cur_idx]) continue;
                ncc_vec_l[(dy+offset)*ncc_wnd_sz + (dx+offset)] = m_gray_l[cur_idx] - mean_l;
            }
        }

        mcosts[k-st_k] = qing_ncc_value(ncc_vec_l, ncc_vec_r);
    }
}

void StereoFlow::calc_init_disparity(const vector<int> &pre_bestk_l, const vector<int> &pre_bestk_r) {

    m_disp_l.resize(m_total);   memset(&m_disp_l.front(), 0, sizeof(float) * m_total);
    m_disp_r.resize(m_total);   memset(&m_disp_r.front(), 0, sizeof(float) * m_total);

    cout << "\n\tcalculate initial disparity , " << m_min_disp << "~" << m_max_disp << ", disp range = " << m_disp_ranges ;
    cout << "  using disparity result of low resolution layer " << endl;

    m_cost_mtd = set_cost_type(CCNAME);
    m_aggr_mtd = set_aggr_type(CANAME);

    int pre_w = m_w * 0.5;
    int pre_h = m_h * 0.5;

    //calc_init_disparity_l(pre_disp_l);
    QingTimer timer;
    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx = y * m_w + x;
            if(0==m_mask_l[idx]) continue;

            int pre_y = y * 0.5;
            int pre_x = x * 0.5;
            int pre_idx = pre_y * pre_w + pre_x;
            int pre_k = pre_bestk_l[pre_idx];

            int st_k = pre_k ? max(2 * pre_k - 2, 1) : 1;
            int ed_k = pre_k ? min(2 * pre_k + 2, m_disp_ranges) : m_disp_ranges;
            int range = ed_k - st_k;

            vector<float> mcosts(range + 1);  memset(&mcosts.front(), -1.f, sizeof(float)*(range+1));
            calc_cluster_costs_l(st_k, ed_k, x, y, mcosts);

            int maxk;
            float max_mcost, sec_mcost;
            cluster_max_takes_all(mcosts, maxk, max_mcost, sec_mcost);

            maxk += st_k;

            m_disp_l[idx] = qing_k_2_disp(m_max_disp, m_min_disp, maxk);
            m_best_k_l[idx] = maxk;
            m_best_mcost_l[idx] = max_mcost;
            m_best_prior_l[idx] = calc_max_prior(max_mcost, sec_mcost);
        }
    }
    cout << "\tinitial disparity of left image done..." << timer.duration() << "s" << endl;

    //calc_init_disparity_r(pre_disp_r)
    timer.restart();
    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx = y * m_w + x;
            if(0==m_mask_r[idx]) continue;

            int pre_y = y * 0.5;
            int pre_x = x * 0.5f;
            int pre_idx = pre_y * pre_w + pre_x;
            int pre_k = pre_bestk_r[pre_idx];

            int st_k = pre_k ? max(2 * pre_k - 2, 1) : 1;
            int ed_k = pre_k ? min(2 * pre_k + 2, m_disp_ranges) : m_disp_ranges;
            int range = ed_k - st_k;

            vector<float> mcosts(range+1);  memset(&mcosts.front(), -1.f, sizeof(float)*(range+1));
            calc_cluster_costs_r(st_k, ed_k, x, y, mcosts);

            int maxk = 0;
            float max_mcost, sec_mcost;
            cluster_max_takes_all(mcosts, maxk, max_mcost, sec_mcost);

            maxk += st_k;

            m_disp_r[idx] = qing_k_2_disp(m_max_disp, m_min_disp, maxk);
            m_best_k_r[idx] = maxk;
            m_best_mcost_r[idx] = max_mcost;
            m_best_prior_r[idx] = calc_max_prior(max_mcost, sec_mcost);
        }
    }
    cout << "\tinitial disparity of right image done..." << timer.duration() << "s" << endl;
}

void StereoFlow::calc_seed_disparity() {                                                   //calculate seed disparity

    cout << "\n\tselect disparity disparity seeds." << endl;
    cout << "\tcross-check constraint. prior constraint." << endl;
    cout << "\tncc_threshold = " << c_thresh_zncc << ", " << "prior_threshold = " << c_thresh_prior ;

    cross_validation();

    m_seed_disp.resize(m_total);
    memset(&m_seed_disp.front(), 0, sizeof(float)*m_total);

    int cnt = 0;
    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx = y * m_w + x;
            if( 0==m_mask_l[idx] ) continue;
            if( m_best_mcost[idx] > c_thresh_zncc && m_best_prior[idx] > c_thresh_prior )
            {
                m_seed_disp[idx] = m_disp[idx];
                cnt ++;
            }
        }
    }
    cout << '\t' << cnt << " seeds / " << max(m_valid_pixels_l, m_valid_pixels_r) << " pixels.";

    //remove isolated seed matches
    //divide image into a set of 11x11 squares, at least 2 seeds should be inside, otherwise remove seeds in this square
    cnt = removal_isolated_seeds(11, 11);
    cout << "\n\tafter isolated seeds removal: " << cnt << " seeds. " << endl;
    cnt = removal_boundary_seeds(11);
    cout << "\n\tafter boundary seeds removal: " << cnt << " seeds. " << endl;
}

int StereoFlow::removal_isolated_seeds(const int& rsize, const int& thresh) {
    int sw = (m_w+rsize-1)/rsize;
    int sh = (m_h+rsize-1)/rsize;
    int stotal = sh * sw, sidx, idx = -1, cnt = 0;

    // cout << "\t" << m_w << " x " << m_h << " -> devided into " << sw << " x " << sh << endl;
    vector<int> seeds_cnt(stotal, 0);
    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            if (0 == m_mask_l[++idx]) continue;
            if (0 == m_seed_disp[idx]) continue;

            int sx = x / rsize;
            int sy = y / rsize;
            sidx = sy * sw + sx;
            seeds_cnt[sidx]++;
        }
    }

    sidx = -1; cnt = 0;
    for(int sy = 0; sy < sh; ++sy) {
        for(int sx = 0; sx < sw; ++sx) {
            if(seeds_cnt[++sidx] < 10) {
                for(int y = sy * rsize; y < (sy+1) * rsize; ++y ) {
                    for(int x = sx * rsize; x < (sx+1) * rsize; ++x) {
                        m_seed_disp[y*m_w+x] = 0.f;
                    }
                }
            }
            else{
                cnt += seeds_cnt[sidx];
            }
        }
    }
    return cnt;
}

int StereoFlow::removal_boundary_seeds(const int &rsize) {
    int idx = -1, offset = 0, cnt = 0;
    for(int y = 0; y < m_h; ++y ) {
        for(int x = 0; x < m_w; ++x) {

            if (0 == m_seed_disp[++idx]) continue;

            offset = 0;
            while (offset < rsize) {
                if (0 == m_mask_l[idx + offset] || 0 == m_mask_l[idx - offset]) break;
                offset++;
            }
            if (offset < rsize) m_seed_disp[idx] = 0;
            else cnt++;
        }
    }
    return cnt;
}

void StereoFlow::cross_validation() {
    cout << "\n\tcross validation..." << endl;

    m_disp.resize(m_total); memset(&m_disp.front(), 0, sizeof(float)*m_total);
    m_best_k.resize(m_total); memset(&m_best_k.front(), 0, sizeof(int)*m_total);
    m_best_mcost.resize(m_total); memset(&m_best_mcost.front(), 0, sizeof(float)*m_total);
    m_best_prior.resize(m_total); memset(&m_best_prior.front(), 0, sizeof(float)*m_total);

    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx_l = y * m_w + x;

            if(0==m_mask_l[idx_l] || 0==m_disp_l[idx_l]) continue;

            int idx_r = idx_l - m_disp_l[idx_l];

            if(0>idx_r||m_total<=idx_r) continue;
            if(0==m_mask_r[idx_r] || 0==m_disp_r[idx_r]) continue;

            if( m_disp_l[idx_l] == m_disp_r[idx_r]) {
                m_disp[idx_l] = m_disp_l[idx_l];
                m_best_k[idx_l] = m_best_k_l[idx_l];
                m_best_mcost[idx_l] = m_best_mcost_l[idx_l];
                m_best_prior[idx_l] = m_best_prior_l[idx_l];
            }
        }
    }
}

void StereoFlow::disp_2_matches() {
    m_matches_l.clear(); m_matches_l.reserve(m_total * 0.2);
    m_matches_r.clear(); m_matches_r.reserve(m_total * 0.2);

    for(int y = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            int idx = y * m_w + x;
            if(0==m_mask_l[idx] || 0.f==m_seed_disp[idx]) continue;
            float d = m_seed_disp[idx];
            float lx = x, ly = y, rx = lx - d, ry = ly;
            float mcost = m_best_mcost[idx];
            float prior = m_best_prior[idx];

            m_matches_l.push_back(Match(lx, ly, d, mcost, prior ));
            m_matches_r.push_back(Match(rx, ry, d, mcost, prior ));
        }
    }
    cout << "\tseed disparity to matches done..." << endl;
}

void StereoFlow::matches_2_disp() {
    memset(&m_disp_l.front(), 0.f, sizeof(float) * m_total);
    memset(&m_disp_r.front(), 0.f, sizeof(float) * m_total);

    for(int i = 0, sz = m_matches_l.size(); i < sz; ++i) {
        Match tmatch = m_matches_l[i];
        float lx = tmatch.get_x();
        float ly = tmatch.get_y();
        float d  = tmatch.get_d();
        float mcost = tmatch.get_mcost();
        float prior = tmatch.get_prior();

        int idx = ly * m_w + lx;
        m_disp_l[idx] = d;
        m_best_k_l[idx] = qing_disp_2_k(m_max_disp, m_min_disp, d);
        m_best_mcost_l[idx] = mcost;
        m_best_prior_l[idx] = prior;
    }

    for(int i = 0, sz = m_matches_r.size(); i < sz; ++i) {
        Match tmatch = m_matches_r[i];
        float rx = tmatch.get_x();
        float ry = tmatch.get_y();
        float d  = tmatch.get_d();
        float mcost = tmatch.get_mcost();
        float prior = tmatch.get_prior();

        int idx = ry * m_w + rx;
        m_disp_r[idx] = d;
        m_best_k_r[idx] = qing_disp_2_k(m_max_disp, m_min_disp, d);
        m_best_mcost_r[idx] = mcost;
        m_best_prior_r[idx] = prior;
    }
    cout << "\n\tmatches to disparity done...." << endl;
}

void StereoFlow::propagate(const int direction, priority_queue<Match> &queue, MatchHash *&hashmap) {

    while(!queue.empty()) {
        Match t_match = queue.top();
        queue.pop();

        float d = t_match.get_d(), lx, ly, rx, ry;
        int k = qing_disp_2_k(m_max_disp, m_min_disp, d);
        if(0>k || m_disp_ranges<k) continue;

        if(0==direction) {
            lx = t_match.get_x();
            ly = t_match.get_y();
            rx = lx - d;
            ry = ly;
        }
        else {
            rx = t_match.get_x();
            ry = t_match.get_y();
            lx = rx + d;
            ly = ry;
        }

        Point2f t_key(t_match.get_x(), t_match.get_y());
        MatchValue t_value(t_match.get_d(), t_match.get_mcost(), t_match.get_prior());

        if( false == hashmap->max_store(t_key, t_value, 1) ) continue;
        //hashmap->parse();

        int cnt = 0, offset = 1, total = (2*offset+1)*(2*offset+1), cur_l_idx, cur_r_idx;
        float cur_lx, cur_ly, cur_rx, cur_ry;
        vector<Match> neighbors(0);
        neighbors.reserve(total);                                  //neighbor matches

        for(int dy = -offset; dy <= offset; ++dy) {
            cur_ly = ly + dy;
            cur_ry = ry + dy;
            if(0 > cur_ly || m_h <= cur_ly || 0 > cur_ry || m_h <= cur_ry) continue;

            for(int dx = -offset; dx <= offset; ++dx) {
                if(0==dy && 0==dx) continue;

                cur_lx = lx + dx;
                cur_rx = rx + dx;
                if(0 > cur_lx || m_w <= cur_lx || 0 > cur_rx || m_w <= cur_rx) continue;

                cur_l_idx = cur_ly * m_w + cur_lx;
                cur_r_idx = cur_ry * m_w + cur_rx;

                if(0==m_mask_l[cur_l_idx] || 0==m_mask_r[cur_r_idx]) continue;

                vector<float> mcosts(2*offset + 1, -1.f);

                if(0==direction)
                    calc_cluster_costs_l(k-1, k+1, cur_lx, cur_ly, mcosts);
                else
                    calc_cluster_costs_r(k-1, k+1, cur_rx, cur_ry, mcosts);

                int maxk;
                float max_mcost, sec_mcost, prior;
                cluster_max_takes_all(mcosts, maxk, max_mcost, sec_mcost);
                prior = calc_max_prior(max_mcost, sec_mcost);

                //   //  cout << max_mcost << '\t' << prior << endl;

                if(max_mcost >= c_thresh_e_zncc && prior >= c_thresh_e_prior) {
                    float d = qing_k_2_disp(m_max_disp, m_min_disp, maxk+k-1);

                    if(0==direction)
                        neighbors.push_back(Match(cur_lx, cur_ly, d, max_mcost, prior));
                    else
                        neighbors.push_back(Match(cur_rx, cur_ry, d, max_mcost, prior));

                    cnt++;
                }
            }
        }

        if(cnt * 2 >= total) {
            for(int i = 0; i < cnt; ++i) queue.push(neighbors[i]);
        }
    }
}

//propagate
void StereoFlow::seed_propagate(const int direction) {

    if(0==direction)  //left->right
        cout << "\n\tpropagation in left image according best-first ncc strategy..."  ;
    else
        cout << "\n\tpropagation in right image according best-first ncc strategy..." ;

    MatchHash * hashmap = new MatchHash();
    if(NULL==hashmap) {
        cerr << "failed to new a match hash map..." << endl;
        exit(-1);
    }

    vector<Match> seeds(0);
    if(0==direction) {
        seeds.resize(m_matches_l.size());
        copy(m_matches_l.begin(), m_matches_l.end(), seeds.begin());
    }
    else {
        seeds.resize(m_matches_r.size());
        copy(m_matches_r.begin(), m_matches_r.end(), seeds.begin());
    }

    cout << "\tstart propagation..." ;
    priority_queue<Match> prior_queue;
    for(int i = 0, sz = seeds.size(); i < sz; ++i) {
        prior_queue.push(seeds[i]);
    }
    cout << "\t" << prior_queue.size() << " matches..." << endl;

#if 0
    fstream fout("prior_queue_match.txt", ios::out);
    for(int i = 0, sz = prior_queue.size(); i < sz; ++i) {
        Match t_match = prior_queue.top();
        prior_queue.pop();
        fout << t_match ;
    }
#endif

    hashmap->init(m_total, m_total);
    cout << "\tmatch hash map initialization done..." << endl;  //<< m_num_keys << " keys, " << m_num_values << " values. " << endl;

    propagate(direction, prior_queue, hashmap);
    cout << "\tafter propagation: " << hashmap->get_num_of_keys() << " matches..." << endl;

    if(0==direction)    hashmap->copy_to(m_matches_l);
    else hashmap->copy_to(m_matches_r);
}

void StereoFlow::copy_disp_2_disp_l() {
    m_disp_l.clear(); m_disp_l.resize(m_total); copy(m_disp.begin(), m_disp.end(), m_disp_l.begin());
    m_best_k_l.clear(); m_best_k_l.resize(m_total); copy(m_best_k.begin(), m_best_k.end(), m_best_k_l.begin());
    m_best_mcost_l.clear(); m_best_mcost_l.resize(m_total); copy(m_best_mcost.begin(), m_best_mcost.end(), m_best_mcost_l.begin());
    m_best_prior_l.clear(); m_best_prior_l.resize(m_total); copy(m_best_prior.begin(), m_best_prior.end(), m_best_prior_l.begin());
}

void StereoFlow::copy_disp_2_disp_r() {
    m_disp_r.clear(); m_disp_r.resize(m_total); memset(&m_disp_r.front(), 0.f, sizeof(float)*m_total);
    m_best_k_r.clear(); m_best_k_r.resize(m_total); memset(&m_best_k_r.front(), 0, sizeof(int)*m_total);
    m_best_mcost_r.clear(); m_best_mcost_r.resize(m_total); memset(&m_best_mcost_r.front(), -1.f, sizeof(float)*m_total);
    m_best_prior_r.clear(); m_best_prior_r.resize(m_total); memset(&m_best_prior_r.front(),  0.f, sizeof(float)*m_total);

    for(int idx_l = 0; idx_l < m_total; ++idx_l) {
        if(0==m_mask_l[idx_l] || 0==m_disp[idx_l]) continue;

        float d = m_disp[idx_l];
        int idx_r = idx_l - d;

        if(0==m_mask_r[idx_r]) continue;
        m_disp_r[idx_r] = d;
        m_best_k_r[idx_r] = m_best_k[idx_l];
        m_best_mcost_r[idx_r] = m_best_mcost[idx_l];
        m_best_prior_r[idx_r] = m_best_prior[idx_l];
    }

}

void StereoFlow::calc_rematch_borders(const vector<float> &disp, const vector<uchar> &mask, const int scanline, vector<int> &border0, vector<int> &border1) {

# if 0
    Mat disp_img, disp_bgr_img;
    uchar * ptr;
    if(0 == scanline) {
        cout << "scanline == " << scanline << endl;
        disp_img.create(m_h, m_w, CV_8UC1);
        qing_float_vec_2_uchar_img(disp, 1, disp_img);
        disp_bgr_img.create(m_h, m_w, CV_8UC3);
        cvtColor(disp_img, disp_bgr_img, CV_GRAY2BGR);
        ptr = (uchar*)disp_bgr_img.ptr<uchar>(0);
    }
# endif

    for(int x = 0; x < m_w; ++x) {
        int idx = scanline * m_w + x;
        if(0==mask[idx]) continue;
        if(0==disp[idx]) {
            int pre_x = x - 1, pre_idx = idx - 1;
            while(0 <= pre_x) {
                if(0!=mask[pre_idx] && 0==disp[pre_idx]) {
                    pre_x--;
                    pre_idx--;
                }
                else break;
            }
            if(0 > pre_x || (0 <= pre_x && 0 == mask[pre_idx]))
                pre_x = -1;

            int pos_x = x + 1;
            int pos_idx = idx + 1;
            while(m_w > pos_x) {
                if(0!=mask[pos_idx] && 0==disp[pos_idx]) {
                    pos_x++;
                    pos_idx++;
                }
                else break;
            }
            if(m_w <= pos_x || (m_w > pos_x && 0 == mask[pos_idx]))
                pos_x = - 1;

            border0[x] = pre_x;
            border1[x] = pos_x;
        }
    }

#if 0
    if(0==scanline) {
        cout << "scanline == " << scanline << endl;
        for(int x = 0; x < m_w; ++x) {
            int idx = scanline * m_w + x;
            if(0==mask[idx]||0!=disp[idx]) continue;
            if(border0[x] == -1 || border1[x] == -1) {
                ptr[3*idx + 0] = 0;
                ptr[3*idx + 1] = 0;
                ptr[3*idx + 2] = 255;
            }
            else {
                ptr[3*idx + 0] = 255;
                ptr[3*idx + 1] = 0;
                ptr[3*idx + 2] = 0;

            }
        }
        imshow("test_rematch_border", disp_bgr_img);
        waitKey(0);
        destroyWindow("test_rematch_border");
    }
#endif
}

void StereoFlow::re_match_l() {
    copy_disp_2_disp_l();
    for(int y = 0, idx = 0; y < m_h; ++y) {
        vector<int> border0(m_w, -1);
        vector<int> border1(m_w, -1);
        calc_rematch_borders(m_disp_l, m_mask_l, y, border0, border1);

        for(int x = 0; x < m_w; ++x) {
            if(0==m_mask_l[idx] || 0!=m_disp_l[idx]) {idx++;continue;}
            int st_k, ed_k, range;
            if(-1==border0[x] || -1==border1[x]) {
                st_k = 1; ed_k = m_disp_ranges;
            }
            else {
                st_k = m_best_k_l[ y * m_w + border0[x] ];
                ed_k = m_best_k_l[ y * m_w + border1[x] ];
                if(st_k > ed_k) swap(st_k, ed_k);
            }
            range = ed_k - st_k;

            vector<float> mcosts(range+1, -1.f);
            calc_cluster_costs_l(st_k, ed_k, x, y, mcosts);

            int maxk;
            float max_mcost, sec_mcost;
            cluster_max_takes_all(mcosts, maxk, max_mcost, sec_mcost);

            maxk += st_k;
            m_disp_l[idx] = qing_k_2_disp(m_max_disp, m_min_disp, maxk);
            m_best_k_l[idx] = maxk;
            m_best_mcost_l[idx] = max_mcost;
            m_best_prior_l[idx] = calc_max_prior(max_mcost, sec_mcost);
            idx++;
        }
    }
}

void StereoFlow::re_match_r() {
    copy_disp_2_disp_r();
    for(int y = 0, idx = 0; y < m_h; ++y) {
        vector<int> border0(m_w, -1);
        vector<int> border1(m_w, -1);
        calc_rematch_borders(m_disp_r, m_mask_r, y, border0, border1);

        for(int x = 0; x < m_w; ++x) {
            if(0==m_mask_r[idx] || 0!=m_disp_r[idx]) {idx++; continue;}
            int st_k, ed_k, range;
            if(-1==border0[x] || -1==border1[x]) {
                st_k = 1; ed_k = m_disp_ranges;
            }
            else {
                st_k = m_best_k_r[ y * m_w + border0[x] ];
                ed_k = m_best_k_r[ y * m_w + border1[x] ];
                if(st_k > ed_k) swap(st_k, ed_k);
            }
            range = ed_k - st_k;

            vector<float> mcosts(range+1, -1.f);
            calc_cluster_costs_r(st_k, ed_k, x, y, mcosts);

            int maxk;
            float max_mcost, sec_mcost;
            cluster_max_takes_all(mcosts, maxk, max_mcost, sec_mcost);

            maxk += st_k;
            m_disp_r[idx] = qing_k_2_disp(m_max_disp, m_min_disp, maxk);
            m_best_k_r[idx] = maxk;
            m_best_mcost_r[idx] = max_mcost;
            m_best_prior_r[idx] = calc_max_prior(max_mcost, sec_mcost);
            idx ++;
        }
    }
}

void StereoFlow::upsampling_using_rbf() {
    cout << "\tupsampling left disparities..." ;
    QingTimer timer;
    upsampling_using_rbf(m_mat_view_l, m_mask_l, m_disp_l, m_best_k_l, m_best_mcost_l, m_best_prior);
    cout << "done..." << timer.duration() << "s" << endl;
    cout << "\tupsamling right disparities...";
    timer.restart();
    upsampling_using_rbf(m_mat_view_r, m_mask_r, m_disp_r, m_best_k_r, m_best_mcost_r, m_best_prior);
    cout << "done..." << timer.duration() << "s" << endl;
}

void StereoFlow::upsampling_using_rbf(const Mat& rgb_view, const vector<uchar>& mask,
                                      vector<float>& disp, vector<int>& best_k, vector<float>& best_mcost, vector<float>& best_prior) {

    float sigma_spatial = 0.005f;
    float sigma_range = 0.1f;

    vector<uchar> guidance(m_total * 3);
    vector<uchar> gradient_x(m_total), gradient_y(m_total);                                                                                     //color image gradients
    vector<float> out(m_total), in(m_total), temp(m_total), temp_2w(2*m_w), ones(m_total), ones_temp(m_total), ones_temp_2w(2*m_w);             //disparity

    //rgb image as guidance
    memcpy(&guidance.front(),rgb_view.data, sizeof(uchar)*m_total*3);
    memset(&in.front(), 0, sizeof(float)*m_total);
    copy(disp.begin(), disp.end(), in.begin());
    for(int idx = 0; idx < m_total; ++idx) {
        if(0==mask[idx]) continue;
        if(5.f > in[idx]) in[idx] = 0.f;
    }

    qx_zdepth_upsampling_using_rbf(&out[0], &in[0], &guidance[0], &temp[0], &temp_2w[0], &gradient_x[0], &gradient_y[0],
            &ones[0], &ones_temp[0], &ones_temp_2w[0], m_h, m_w, sigma_spatial, sigma_range);


    memset(&disp.front(), 0.f, sizeof(float)*m_total);
    memset(&best_k.front(), 0, sizeof(int)*m_total);
    memset(&best_mcost.front(), -1, sizeof(float)*m_total);
    memset(&best_prior.front(), 0, sizeof(float)*m_total);
    for(int idx = 0; idx < m_total; idx++) {
        if(0==mask[idx]) continue;
        else
        {
            disp[idx] = out[idx];
            best_k[idx] = qing_disp_2_k(m_max_disp, m_min_disp, disp[idx]);
            best_mcost[idx] = c_thresh_e_zncc;
            best_prior[idx] = c_thresh_e_prior;
        }
    }
# if 0
    Mat disp_img(m_h, m_w, CV_8UC1);
    qing_float_vec_2_uchar_img(disp, m_scale, disp_img);
    imshow("test_unsampling_using_rbf", disp_img);
    waitKey(0);
    destroyWindow("test_unsampling_using_rbf");
# endif
}

void StereoFlow::check_outliers_l() {
    m_outliers_l.clear(); m_outliers_l.resize(m_total, 0);
    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            if(0==m_mask_l[idx]) { idx++; if(m_total<=idx) break; continue; }

            float d_l = m_disp_l[idx];
            int rx = x - d_l;
            int ridx = idx - d_l;

            if( 0>rx || m_w <= rx || 0 > ridx || m_total <= ridx )   { m_outliers_l[idx] = DISP_OCCLUSION; }
            else {
                if(abs(d_l - m_disp_r[ridx]) > DISP_TOLERANCE) {
                    bool is_occluded = true;
                    for(int k = 0; k <= m_disp_ranges; ++k) {
                        float d = qing_k_2_disp(m_max_disp, m_min_disp, k);
                        if((x-d) >= 0 && d == m_disp_r[ridx]) {
                            is_occluded = false;
                            break;
                        }
                    }
                    m_outliers_l[idx] = is_occluded ? DISP_OCCLUSION : DISP_MISMATCH;
                }
            }
            idx++; if(m_total<=idx) break;
        }
    }

# if 0
    Mat disp(m_h, m_w, CV_32FC1);
    Mat disp_img(m_h, m_w, CV_8UC1);
    Mat disp_rgb_img(m_h, m_w, CV_8UC3);
    qing_vec_2_img<float>(m_disp_l, disp);
    disp.convertTo(disp_img, CV_8UC1, m_scale);
    cvtColor(disp_img, disp_rgb_img, CV_GRAY2BGR);

    uchar * ptr_rgb = (uchar *)disp_rgb_img.ptr<uchar>(0);

    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            if(0==m_mask_l[idx] || 0==m_outliers_l[idx]) {idx++;continue;}
            if(DISP_MISMATCH == m_outliers_l[idx]) {                 //red
                ptr_rgb[3*idx+0] = 0; ptr_rgb[3*idx+1] = 0; ptr_rgb[3*idx+2] = 255;
            }
            else if(DISP_OCCLUSION == m_outliers_l[idx]) {           //blue
                ptr_rgb[3*idx+0] = 255; ptr_rgb[3*idx+1] = 0; ptr_rgb[3*idx+2] = 0;
            }
            idx ++;
        }
    }

    imshow("test_outliers_l", disp_rgb_img);
    waitKey(0);
    destroyWindow("test_outliers_l");
# endif
}

void StereoFlow::check_outliers_r() {
    m_outliers_r.clear(); m_outliers_r.resize(m_total, 0);
    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            if(m_mask_r[idx]==0) {idx++; if(m_total<=idx) break; continue;}

            float d_r = m_disp_r[idx];
            int lx = x + d_r;
            int lidx = idx + d_r;

            if( 0>lx || m_w <= lx || 0 > lidx || m_total <= lidx )   { m_outliers_r[idx] = DISP_OCCLUSION; }
            else {
                if( abs(d_r - m_disp_l[lidx]) > DISP_TOLERANCE ) {
                    bool is_occluded = true;
                    for(int k = 0; k <= m_disp_ranges; ++k) {
                        float d = qing_k_2_disp(m_max_disp, m_min_disp, k);
                        if((x+d) <= m_w && d==m_disp_l[lidx]) {
                            is_occluded = false;
                            break;
                        }
                    }
                    m_outliers_r[idx] = is_occluded ? DISP_OCCLUSION : DISP_MISMATCH;
                }
            }
            idx ++;if(m_total<=idx) break;
        }
    }

# if 0
    Mat disp(m_h, m_w, CV_32FC1), disp_img(m_h, m_w, CV_8UC1), disp_rgb_img(m_h, m_w, CV_8UC3);
    qing_vec_2_img<float>(m_disp_r, disp);
    disp.convertTo(disp_img, CV_8UC1, m_scale);
    cvtColor(disp_img, disp_rgb_img, CV_GRAY2BGR);

    uchar * ptr_rgb = (uchar *)disp_rgb_img.ptr<uchar>(0);

    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            if(0==m_mask_r[idx] || 0==m_outliers_r[idx]) {idx++;continue;}
            if(DISP_MISMATCH == m_outliers_r[idx]) {                 //red
                ptr_rgb[3*idx+0] = 0; ptr_rgb[3*idx+1] = 0; ptr_rgb[3*idx+2] = 255;
            }
            else if(DISP_OCCLUSION == m_outliers_r[idx]) {           //blue
                ptr_rgb[3*idx+0] = 255; ptr_rgb[3*idx+1] = 0; ptr_rgb[3*idx+2] = 0;
            }
            idx ++;
        }
    }

    imshow("test_outliers_r", disp_rgb_img);
    waitKey(0);
    destroyWindow("test_outliers_r");
# endif
}

void StereoFlow::init_region_voter() {
    m_region_voter = new RegionVoter();
    cout << "\tregion voting. min_vote_count = " << m_region_voter->get_vote_count_thresh()
         << ", min_vote_ratio = " << m_region_voter->get_vote_ratio_thresh() << endl;
}

void StereoFlow::region_voting() {
    region_voting(0, m_mask_l, m_disp_l, m_best_k_l, m_best_mcost_l, m_best_prior_l, m_outliers_l);
    cout << "\tend of region voting in disp_left..." << endl;
    region_voting(1, m_mask_r, m_disp_r, m_best_k_r, m_best_mcost_r, m_best_prior_r, m_outliers_r);
    cout << "\tend of region voting in disp_right..." << endl;
# if 0
    Mat disp_l(m_h, m_w, CV_32FC1), disp_img_l(m_h, m_w, CV_8UC1);
    qing_vec_2_img<float>(m_disp_l, disp_l);
    disp_l.convertTo(disp_img_l, CV_8UC1, m_scale);

    Mat disp_r(m_h, m_w, CV_32FC1), disp_img_r(m_h, m_w, CV_8UC1);
    qing_vec_2_img<float>(m_disp_r, disp_r);
    disp_r.convertTo(disp_img_r, CV_8UC1, m_scale);
    imshow("regionvoting_disp_l", disp_img_l);
    imshow("regionvoting_disp_r", disp_img_r);
    waitKey(0);
    destroyAllWindows();
# endif
}

void StereoFlow::region_voting(const int direction, const vector<uchar>& mask, vector<float>& disp,
                               vector<int>& best_k, vector<float>& best_mcost, vector<float>& best_prior,
                               vector<uchar>& outliers) {
    vector<float> res_disp(m_total, 0.f); copy(disp.begin(), disp.end(), res_disp.begin());
    vector<int> res_best_k(m_total, 0);  copy(best_k.begin(), best_k.end(),res_best_k.begin());
    vector<float> res_best_mcost(m_total, -1.f); copy(best_mcost.begin(), best_mcost.end(), res_best_mcost.begin());
    vector<float> res_best_prior(m_total, 0.f);  copy(best_prior.begin(), best_prior.end(), res_best_prior.begin());

#if 0
    Mat test_img(m_h, m_w, CV_8UC1);
    qing_float_vec_2_uchar_img(res_disp, m_scale, test_img);
    imshow("test_disp_regionvote", test_img);
    waitKey(0);
    destroyWindow("test_disp_regionvote");
    uchar * ptr = (uchar *)test_img.ptr<uchar>(0);
    for(int i = 0; i < m_total; ++i)
        ptr[i]= res_best_k[i];
    imshow("test_bestk_regionvote", test_img);
    waitKey(0);
    destroyWindow("test_bestk_regionvote");
#endif

    vector<int>& borders_u = direction ? m_support_region->get_u_borders_r() : m_support_region->get_u_borders_l();
    vector<int>& borders_d = direction ? m_support_region->get_d_borders_r() : m_support_region->get_d_borders_l();
    vector<int>& borders_l = direction ? m_support_region->get_l_borders_r() : m_support_region->get_l_borders_l();
    vector<int>& borders_r = direction ? m_support_region->get_r_borders_r() : m_support_region->get_r_borders_l();

    //using res_disp to updating votes
    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {

            if(0==mask[idx] || 0==outliers[idx]) {idx++; continue;}

            vector<float> disp_hist(m_disp_ranges+1, 0.f);
            int lborder = -borders_l[idx];
            int rborder =  borders_r[idx];
            int votes = 0;

            for(int dx = lborder; dx <= rborder; ++dx) {
                int cur_x = x + dx;
                if(0>cur_x || m_w <= cur_x) continue;

                int cur_idx = idx + dx;
                int uborder = -borders_u[cur_idx];
                int dborder =  borders_d[cur_idx];

                for(int dy = uborder; dy <= dborder; ++dy) {
                    int cur_y = y + dy;
                    if(0>cur_y || m_h <= cur_y) continue;

                    cur_idx = cur_y * m_w + cur_x;

                    if(0==mask[cur_idx] || 0.f==res_disp[cur_idx]) continue;        //invalid pixels can not to vote
                    if(0!=outliers[cur_idx]) continue;                                //outliers can not to vote

                    float d = res_disp[cur_idx];
                    int k = qing_disp_2_k(m_max_disp, m_min_disp, d);

                    if(0>k || disp_hist.size() <= k) { /*cout << "k=" << k << endl;*/ continue;}
                    votes++;
                    disp_hist[k]++;
                }
            }

            // is the number of vote sufficient
            // not sufficient;
            if(m_region_voter->get_vote_count_thresh() >= votes) {idx++; continue;}

            float new_d = res_disp[idx];  // cout << "yes. " << votes << endl;

            int new_best_k = 0, max_vote_cnt = 0;
            // float vote_ratio = 0.f, max_vote_ratio = 0.f;
            for(int k = 0; k <= m_disp_ranges; ++k) {
                int vote_cnt = disp_hist[k];
                //  cout << "k = " << k << ", vote_cnt = " << vote_cnt << endl;
                if(vote_cnt > max_vote_cnt) {
                    max_vote_cnt = vote_cnt;
                    new_best_k = k;
                    new_d = qing_k_2_disp(m_max_disp, m_min_disp, k);
                }
                disp_hist[k] = 0;
            }

            outliers[idx] = 0;
            res_disp[idx] = new_d;
            res_best_k[idx] = new_best_k;
            res_best_mcost[idx] = c_thresh_e_zncc;
            res_best_prior[idx] = c_thresh_e_prior;
            idx ++;
        }
    }
    copy(res_disp.begin(), res_disp.end(), disp.begin());
    copy(res_best_k.begin(), res_best_k.end(), best_k.begin());
    copy(res_best_mcost.begin(), res_best_mcost.end(), best_mcost.begin());
    copy(res_best_prior.begin(), res_best_prior.end(), best_prior.begin());
}

void StereoFlow::median_filter() {
    vector<int> new_best_k_l(0), new_best_k_r(0);

#if 0
    Mat test_img(m_h, m_w, CV_8UC1);
    uchar * ptr = (uchar *)test_img.ptr<uchar>(0);
    for(int i = 0; i < m_total; ++i)
        ptr[i]= m_best_k_l[i];
    imshow("test", test_img);
    waitKey(0);
    destroyWindow("test");
#endif

    qing_median_filter(new_best_k_l, m_best_k_l, m_mask_l, m_h, m_w, m_wnd_size, m_disp_ranges+1);
    qing_median_filter(new_best_k_r, m_best_k_r, m_mask_r, m_h, m_w, m_wnd_size, m_disp_ranges+1);

    copy(new_best_k_l.begin(), new_best_k_l.end(), m_best_k_l.begin());
    for(int idx = 0; idx < m_total; ++idx) {
        if(0==m_mask_l[idx])continue;
        m_disp_l[idx] = qing_k_2_disp(m_max_disp, m_min_disp, m_best_k_l[idx]);
        m_best_mcost_l[idx] = c_thresh_e_zncc;
        m_best_prior_l[idx] = c_thresh_e_prior;
    }

    copy(new_best_k_r.begin(), new_best_k_r.end(), m_best_k_r.begin());
    for(int idx = 0; idx < m_total; ++idx) {
        if(0==m_mask_r[idx])continue;
        m_disp_r[idx] = qing_k_2_disp(m_max_disp, m_min_disp, m_best_k_r[idx]);
        m_best_mcost_r[idx] = c_thresh_e_zncc;
        m_best_prior_r[idx] = c_thresh_e_prior;
    }
}

//using formula
void StereoFlow::subpixel_enhancement() {
    for(int y = 0, idx = 0; y < m_h; ++y) {
        for(int x = 0; x < m_w; ++x) {
            if(0==m_mask_l[idx] || 0.f == m_disp[idx]) continue;

            float d = m_disp[idx];
            int lx = x, ly = y;

            if( 0>(x-d) || 0==m_mask_r[idx - d] ) continue;

            int st_k = qing_disp_2_k(m_max_disp, m_min_disp, d-1);
            int ed_k = qing_disp_2_k(m_max_disp, m_min_disp, d+1);
            int range = ed_k - st_k + 1;

            vector<float> mcosts(range, -1.f);
            calc_cluster_costs_l(st_k, ed_k, lx, ly, mcosts);

            double sub_mcost = mcosts[0];
            double mcost = mcosts[1];
            double add_mcost = mcosts[2];
            double fenmu = (add_mcost + sub_mcost - 2 * mcost);
            if(fenmu != 0) {
                m_disp_l[idx] = d - 0.5 * (add_mcost - sub_mcost) / fenmu;
            }
        }
    }
}

//2017.05.31
void StereoFlow::matching_cost() {
    m_hwd_costvol_l.clear();
    m_hwd_costvol_r.clear();

    qing_allocf_3(m_hwd_costvol_l, m_h, m_w, m_disp_ranges+1);
    matching_cost_from_zncc();
# if 0
    string mcost_folder = "./matching-cost-ncc/";
    qing_create_dir(mcost_folder);
    string filename ;
    float * temp_mcost = new float[m_total];
    for(int d = 0; d < m_disp_ranges+1; ++d) {
        memset(temp_mcost, 0, sizeof(float)*m_total);
        for(int y = 0, idx = 0; y < m_h; ++y) {
            for(int x = 0; x < m_w; ++x) {
                temp_mcost[idx++] = m_hwd_costvol_l[y][x][d];
            }
        }
        filename = mcost_folder + "zncc_" + qing_int_2_string(d) + ".txt";
        qing_save_mcost_txt(filename, temp_mcost, m_total, m_w);
    }
# endif
    qing_allocf_3(m_hwd_costvol_r, m_h, m_w, m_disp_ranges+1);
    qing_stereo_flip_cost_vol(m_hwd_costvol_r, m_hwd_costvol_l, m_h, m_w, m_disp_ranges+1);

# if 1
    qing_depth_max_cost(m_disp_l, m_hwd_costvol_l, m_h, m_w, m_disp_ranges + 1);
    Mat uimg(m_h, m_w, CV_8UC1, Scalar(0));
    qing_float_vec_2_uchar_img(m_disp_l, m_scale, uimg);
    imwrite("mcost_disp_l.jpg", uimg);
    qing_depth_max_cost(m_disp_r, m_hwd_costvol_r, m_h, m_w, m_disp_ranges + 1);
    qing_float_vec_2_uchar_img(m_disp_r, m_scale, uimg);
    imwrite("mcost_disp_r.jpg", uimg);
# endif
}

void StereoFlow::matching_cost_from_zncc() {

    //pre-computation of ncc vectors
    cout << m_wnd_size << endl;
    int wndsz2 = m_wnd_size * m_wnd_size;
    m_ncc_vecs_l.clear(); qing_allocf_3(m_ncc_vecs_l, m_h, m_w, wndsz2);
    m_ncc_vecs_r.clear(); qing_allocf_3(m_ncc_vecs_r, m_h, m_w, wndsz2);

    qing_compute_ncc_vecs(m_ncc_vecs_l, m_gray_l, m_mean_l, m_mask_l, m_h, m_w, m_wnd_size);
    qing_compute_ncc_vecs(m_ncc_vecs_r, m_gray_r, m_mean_r, m_mask_r, m_h, m_w, m_wnd_size);

    for(int i = 0; i <= m_disp_ranges; ++i) {
        for(int y = 0; y < m_h; ++y) {
            for(int x = 0; x < i; ++x) {
                m_hwd_costvol_l[y][x][i] = qing_ncc_value(m_ncc_vecs_l[y][x], m_ncc_vecs_r[y][0]);
            }
            for(int x = i; x < m_w; ++x) {
                m_hwd_costvol_l[y][x][i] = qing_ncc_value(m_ncc_vecs_l[y][x], m_ncc_vecs_r[y][x - i]);

# if 0
                if(y==6&&i==14&&x>120&&x<140) {
                    cout << x << ": " << endl;
                    for(int m = 0; m < wndsz2 ; ++m) {
                        cout << m_ncc_vecs_l[y][x][m] << ' ' ;
                    }
                    cout << endl;
                    for(int m = 0; m < wndsz2 ; ++m) {
                        cout << m_ncc_vecs_r[y][x-i][m] << ' ' ;
                    }
                    cout << endl;
                    cout << m_hwd_costvol_l[y][x][i] << endl << endl;
                }
# endif
            }
        }
    }
}

void StereoFlow::beeler_disp_refinement() {
    int cnt = 0, times = 50, idx;
    float tstep = 0.5f, refine_disp;
    double ddata, dsmooth, wdata, wsmooth, epsilon;
    QingTimer timer;

    while ( cnt <= times ) {
        cout << cnt << "th iteration.\t";
        timer.restart();
        if (cnt >= (int)(times * 0.5))
            tstep =  max(0.5f - 0.02f * cnt, 0.1f);

        for(int y = 1; y < m_h-1; y ++ ) {
            for( int x = 1; x < m_w-1; x ++ ) {
                idx = y * m_w + x;
                if (255 != m_mask_l[idx]) { m_disp_l[idx] = 0.f; continue; }
                if (255 != m_mask_l[idx-1] || 255 != m_mask_l[idx+1] ||
                        255 != m_mask_l[idx-m_w] || 255 != m_mask_l[idx+m_w] ) continue;

                epsilon = compute_data_item(ddata, wdata, y, x, tstep);
                if (true == compute_smooth_item(dsmooth, wsmooth, y,x,epsilon) )
                {
                    refine_disp = ((wdata*ddata)+(wsmooth*dsmooth) ) / (wdata + wsmooth);
                    m_disp_l[idx] = refine_disp;
                }
            }
        }
        cout << timer.duration() << "s. " << endl;
        cnt ++;
    }
}

double StereoFlow::compute_data_item(double& ddata, double& dweight, const int& y, const int& x, const float& tstep) {
    int idx = y * m_w + x;
    float d = m_disp_l[idx], sub_d = d  - 1, add_d = d  + 1;

    double sub_epsilon = (1 - qing_ncc_value(m_ncc_vecs_l[y][x], m_ncc_vecs_r[y][(int)(x-sub_d)]  ) )* 0.5;
    double src_epsilon = (1 - qing_ncc_value(m_ncc_vecs_l[y][x], m_ncc_vecs_r[y][(int)(x-d)] ) ) * 0.5;
    double add_epsilon = (1 - qing_ncc_value(m_ncc_vecs_l[y][x], m_ncc_vecs_r[y][(int)(x-add_d)]) ) * 0.5;

    assert(sub_epsilon > 0 && sub_epsilon < 1);
    assert(src_epsilon > 0 && src_epsilon < 1);
    assert(add_epsilon > 0 && add_epsilon < 1);

    double min_epsilon = min(src_epsilon, min(sub_epsilon, add_epsilon) );
    if (min_epsilon == sub_epsilon) //minus < src, add
    {
        ddata = d - tstep;
        dweight = src_epsilon - sub_epsilon;
    }
    else if (min_epsilon == src_epsilon) //src < minus, add
    {
        double temp = sub_epsilon + add_epsilon - 2 * src_epsilon;
        ddata = d + tstep * ( (sub_epsilon - add_epsilon) / (temp + 0.1) ); //·ÖÄ¸+0.1ÊÇÎªÁË·ÀÖ¹³ý0²Ù×÷
        dweight = 0.5 * temp;
    }

    else // minepsilon == epsilonadd
    {
        ddata = d + tstep;
        dweight = src_epsilon - add_epsilon;
    }
    return src_epsilon;
}

bool StereoFlow::compute_smooth_item(double& sdata, double& sweight, const int& y, const int& x,  const double& epsilon) {
    double alpha = 1.5;
    double beta = 2.0;
    double wsmooth = 0.005; //2) double wsmooth = 0.1

    int idx = y * m_w + x;

    float d = m_disp_l[idx];
    float u_d = m_disp_l[idx - m_w];
    float d_d = m_disp_l[idx + m_w];
    float l_d = m_disp_l[idx - 1];
    float r_d = m_disp_l[idx + 1];

    double wx, wy;

    wx = abs(l_d - d) - abs(r_d - d);
    wx = exp(- (wx * wx) );
    wy = abs(u_d - d) - abs(d_d - d);
    wy = exp(- (wy * wy) );

    if ( (wx + wy == 0) )
    {
        /*1)
        wx = wx + 0.05;
        wy = wy + 0.05;*/
        return false;
    }

    sdata = wx * (l_d + r_d) + wy * (u_d + d_d);
    sdata = sdata / ( 2 * (wx + wy) );
    sweight = wsmooth * (1 + alpha * epsilon + beta * epsilon * epsilon);
    return true;
}

#if 0
void StereoFlow::scanline_optimize(const int direction) {

}
#endif


