#include "stereo_flow.h"
#include "cost/zncc.h"
#include "cost/census.h"
#include "cost/tad.h"
#include "aggr/bf_aggr.h"
#include "aggr/box_aggr.h"
#include "aggr/gf_aggr.h"
#include "hashmap.h"

#include "../../../Qing/qing_disp.h"
#include "../../../Qing/qing_timer.h"
#include "../../../Qing/qing_image.h"

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

    Mat grayL(m_h, m_w, CV_32FC1); qing_vec_2_img<float>(m_gray_l, grayL);
    Mat grayR(m_h, m_w, CV_32FC1); qing_vec_2_img<float>(m_gray_r, grayR);
    Mat meanL, meanR;
    cv::blur(grayL, meanL, Size(m_wnd_size, m_wnd_size));
    cv::blur(grayR, meanR, Size(m_wnd_size, m_wnd_size));

    qing_img_2_vec<float>(meanL, m_mean_l);
    qing_img_2_vec<float>(meanR, m_mean_r);

# if 1
    cv::blur(grayL, meanL, Size(DEFAULT_NCC_WND, DEFAULT_NCC_WND));
    cv::blur(grayR, meanR, Size(DEFAULT_NCC_WND, DEFAULT_NCC_WND));
    m_ncc_mean_l.resize(m_total);  qing_img_2_vec<float>(meanL, m_ncc_mean_l);
    m_ncc_mean_r.resize(m_total);  qing_img_2_vec<float>(meanR, m_ncc_mean_r);
    cout << "\tmean image with default_ncc_wnd done...." << endl;
# endif
}

void StereoFlow::calc_support_region() {
    m_support_region->calc_patch_borders(m_view_l, m_view_r, m_mask_l, m_mask_r );
}

void StereoFlow::calc_cost_vol() {
    if(m_cost_mtd) {
# if 1
        cout << "\tusing ncc mean image...." << endl;
        m_cost_mtd->build_cost_vol_l(m_gray_l, m_gray_r, m_ncc_mean_l, m_ncc_mean_r, m_mask_l, m_mask_r, m_cost_vol_l);
        m_cost_mtd->build_cost_vol_r(m_gray_l, m_gray_r, m_ncc_mean_l, m_ncc_mean_r, m_mask_l, m_mask_r, m_cost_vol_r);
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
        m_cost_vol_l[k].resize(m_total); memset(&m_cost_vol_l[k].front(), 0, sizeof(float)*m_total);
        m_cost_vol_r[k].resize(m_total); memset(&m_cost_vol_r[k].front(), 0, sizeof(float)*m_total);
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

        mcosts[k-st_k] = qing_calc_ncc_value(ncc_vec_l, ncc_vec_r);
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

        mcosts[k-st_k] = qing_calc_ncc_value(ncc_vec_l, ncc_vec_r);
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
            if( /*m_best_mcost[idx] > c_thresh_zncc &&*/ m_best_prior[idx] > c_thresh_prior )
            {
                m_seed_disp[idx] = m_disp[idx];
                cnt ++;
            }
        }
    }

    cout << '\t' << cnt << " seeds / " << max(m_valid_pixels_l, m_valid_pixels_r) << " pixels."<< endl;
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
        Match t_match = queue.top(); queue.pop();

        float d = t_match.get_d(), lx, ly, rx, ry;
        int k = qing_disp_2_k(m_max_disp, m_min_disp, d);

        if(0==direction) {
            lx = t_match.get_x(); ly = t_match.get_y();
            rx = lx - d; ry = ly;
        }
        else {
            rx = t_match.get_x(); ry = t_match.get_y();
            lx = rx + d; ly = ry;
        }

        Point2f t_key(t_match.get_x(), t_match.get_y());
        MatchValue t_value(t_match.get_d(), t_match.get_mcost(), t_match.get_prior());

        if( false == hashmap->max_store(t_key, t_value, 1) ) continue;
        //hashmap->parse();

        int cnt = 0, offset = 1, total = (2*offset+1)*(2*offset+1), cur_l_idx, cur_r_idx;
        float cur_lx, cur_ly, cur_rx, cur_ry;
        vector<Match> neighbors(0);  neighbors.reserve(total);                                  //neighbor matches

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

                vector<float> mcosts(2*offset + 1, 0.f);

                if(0==direction)
                    calc_cluster_costs_l(k-1, k+1, cur_lx, cur_ly, mcosts);
                else
                    calc_cluster_costs_r(k-1, k+1, cur_rx, cur_ry, mcosts);

                int maxk;
                float max_mcost, sec_mcost, prior;
                cluster_max_takes_all(mcosts, maxk, max_mcost, sec_mcost);
                prior = calc_max_prior(max_mcost, sec_mcost);

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

    cout << "\tstart propagation..." << endl;
    priority_queue<Match> prior_queue;
    for(int i = 0, sz = seeds.size(); i < sz; ++i) {
        prior_queue.push(seeds[i]);
    }

#if 0
    fstream fout("prior_queue_match.txt", ios::out);
    for(int i = 0, sz = prior_queue.size(); i < sz; ++i) {
        Match t_match = prior_queue.top();
        prior_queue.pop();
        fout << t_match ;
    }
    exit(1);
#endif

    hashmap->init(m_total, m_total);
    propagate(direction, prior_queue, hashmap);
    cout << "\tafter propagation: " << hashmap->get_num_of_keys() << endl;

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

void StereoFlow::check_outliers() {

}

void StereoFlow::median_filter() {

}

void StereoFlow::subpixel_enhancement() {

}


#if 0
void StereoFlow::scanline_optimize(const int direction) {

}
#endif


