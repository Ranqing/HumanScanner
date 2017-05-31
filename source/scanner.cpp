#include "scanner.h"
#include "stereo_flow.h"
#include "debug.h"

#include "../../Qing/qing_image.h"
#include "../../Qing/qing_ply.h"

bool HumanBodyScanner::init()
{
    if( false == qing_check_file_suffix(m_stereo_file, ".info")) {
        cerr << "invalid stereo info file format..." << endl;
        return false;
    }

    qing_cwd();

    fstream fin(m_stereo_file.c_str(), ios::in);
    if( false == fin.is_open() ) {
        cerr << "failed to open " << m_stereo_file << endl;
        return false;
    }

    fin >> m_stereo_name;
    fin >> m_stereo_idx;
    fin >> m_camL >> m_camR;
    fin >> m_frame_name;
    fin >> m_img_nameL >> m_img_nameR;
    fin >> m_msk_nameL >> m_msk_nameR;
    fin >> m_crop_pointL.x >> m_crop_pointL.y;
    fin >> m_crop_pointR.x >> m_crop_pointR.y;
    fin >> m_size.width >> m_size.height;
    fin >> m_max_disp;
    fin >> m_min_disp;

    //m_max_disp = 640;                                       //for guarantee
    cout << m_max_disp << '\t' << '~' << '\t' << m_min_disp << endl;

    m_qmatrix = Mat::zeros(4, 4, CV_64FC1);
    double * ptr = (double *)m_qmatrix.ptr<double>(0);
    for(int i = 0; i < 16; ++i) {
        fin >> ptr[i];
    }

    m_out_dir = "./" + m_frame_name;
    qing_create_dir(m_out_dir);
    m_out_dir = m_out_dir + "/" + m_stereo_name;
    qing_create_dir(m_out_dir);

#if DEBUG
    qing_cwd();
    cout << "out dir: " << m_out_dir << endl;
    cout << "crop pointL: " << m_crop_pointL << endl;
    cout << "crop pointR: " << m_crop_pointR << endl;
    cout << "image size: "  << m_size << endl;
    cout << "disp range:"   << m_min_disp << " ~ " << m_max_disp << endl;
    cout << "qmatrix: "     << m_qmatrix  << endl;
    cout << m_img_nameL << endl;
    cout << m_img_nameR << endl;
    cout << m_msk_nameL << endl;
    cout << m_msk_nameR << endl;

    m_debugger = new Debugger(m_out_dir);
#endif

    fin.close();

    //load and crop images
    m_img_nameL = m_img_folder +  m_img_nameL;
    m_img_nameR = m_img_folder +  m_img_nameR;
    m_msk_nameL = m_msk_folder +  m_msk_nameL;
    m_msk_nameR = m_msk_folder +  m_msk_nameR;

    load_and_crop_images();
    cout << "scanner initialization done..." << endl;
}

void HumanBodyScanner::load_and_crop_images()
{
    Mat full_imgL, full_imgR;
    Mat full_mskL, full_mskR;

    //load images
    if(false == qing_load_image(m_img_nameL, CV_LOAD_IMAGE_UNCHANGED, full_imgL)) {
        cerr << "failed to open " << m_img_nameL << endl;
        exit(-1);
    }
    if(false == qing_load_image(m_img_nameR, CV_LOAD_IMAGE_UNCHANGED, full_imgR)) {
        cerr << "failed to open " << m_img_nameR << endl;
        exit(-1);
    }

    //load masks
    if(false == qing_load_image(m_msk_nameL, CV_LOAD_IMAGE_GRAYSCALE, full_mskL) ||
            false == qing_load_image(m_msk_nameR, CV_LOAD_IMAGE_GRAYSCALE, full_mskR)) {
        cout << "no masks here..." << endl;
        m_mskL = Mat(m_size, CV_8UC1, cv::Scalar(255));
        m_mskR = Mat(m_size, CV_8UC1, cv::Scalar(255));
    }
    else {
        //        threshold(full_mskL, full_mskL, 75, 255, THRESH_BINARY);
        //        threshold(full_mskR, full_mskR, 75, 255, THRESH_BINARY);

        qing_threshold_msk(full_mskL, full_mskL, 128, 255);
        qing_threshold_msk(full_mskR, full_mskR, 128, 255);
        m_mskL = Mat::zeros(m_size, CV_8UC1);
        m_mskR = Mat::zeros(m_size, CV_8UC1);
        m_mskL = full_mskL(Rect(m_crop_pointL, m_size)).clone();
        m_mskR = full_mskR(Rect(m_crop_pointR, m_size)).clone();
    }

    //crop images
    full_imgL(Rect(m_crop_pointL, m_size)).copyTo(m_imgL, m_mskL);
    full_imgR(Rect(m_crop_pointR, m_size)).copyTo(m_imgR, m_mskR);

#if DEBUG
    imwrite(m_out_dir + "/crop_imgL.jpg", m_imgL);
    imwrite(m_out_dir + "/crop_imgR.jpg", m_imgR);
    imwrite(m_out_dir + "/crop_mskL.jpg", m_mskL);
    imwrite(m_out_dir + "/crop_mskR.jpg", m_mskR);
#endif

    exit(1);

    //normalize image format and values
    cvtColor(m_imgL, m_imgL, CV_BGR2RGB);
    cvtColor(m_imgR, m_imgR, CV_BGR2RGB);
    m_imgL.convertTo(m_imgL, CV_32F, 1/255.0f);
    m_imgR.convertTo(m_imgR, CV_32F, 1/255.0f);
}

void HumanBodyScanner::build_stereo_pyramid()
{
    cout << "\nstereo pyramid building start." << endl;

    m_max_levels = MAX_LEVELS;
    m_stereo_pyramid = new StereoFlow * [m_max_levels];

    Mat t_imgL = m_imgL.clone();
    Mat t_imgR = m_imgR.clone();
    Mat t_mskL = m_mskL.clone();
    Mat t_mskR = m_mskR.clone();

    int t_max_disp = m_max_disp;
    int t_min_disp = m_min_disp;
    float t_disp_scale = (m_max_disp <= 255) ? (255/m_max_disp) : min((255.f/m_max_disp), 0.5f);

    for(int p = 0; p < m_max_levels; ++p) {
        int w = t_imgL.size().width;
        int h = t_imgR.size().height;

        if( w < MIN_IMG_SIZE || h < MIN_IMG_SIZE || t_max_disp < MIN_DISP_VALUE) {
            m_max_levels = p;
            break;
        }

        cout << "\n\tPyramid " << p << ": "<< t_min_disp << " ~ " << t_max_disp << ", scale = " << t_disp_scale << '\t';
        cout << "initialization of stereo flow..." << endl;
        m_stereo_pyramid[p] = new StereoFlow(t_imgL, t_imgR, t_mskL, t_mskR, t_max_disp, t_min_disp, t_disp_scale);

        //downsample
        t_max_disp = t_max_disp * 0.5;
        t_min_disp = t_min_disp * 0.5;
        t_disp_scale = min(t_disp_scale * 2, 1.f);

#if DEBUG
        if(p) {
            string savefn;

            savefn = m_out_dir + "/crop_imgL_" + int2string(p) + ".jpg";
            qing_save_image(savefn, t_imgL, 255);
            savefn = m_out_dir + "/crop_imgR_" + int2string(p) + ".jpg";
            qing_save_image(savefn, t_imgR, 255);
            savefn = m_out_dir + "/crop_mskL_" + int2string(p) + ".jpg";
            qing_save_image(savefn, t_mskL);
            savefn = m_out_dir + "/crop_mskR_" + int2string(p) + ".jpg";
            qing_save_image(savefn, t_mskR);
        }
#endif
        //guassian filter then choose half cols
        pyrDown(t_imgL, t_imgL);
        pyrDown(t_imgR, t_imgR);
        pyrDown(t_mskL, t_mskL);
        // threshold(t_mskL, t_mskL, 128, 255, THRESH_BINARY);
        qing_threshold_msk(t_mskL, t_mskL, 128, 255);
        pyrDown(t_mskR, t_mskR);
        // threshold(t_mskR, t_mskR, 128, 255, THRESH_BINARY);
        qing_threshold_msk(t_mskR, t_mskR, 128, 255);

        /*setting support area size*/
        /*calculating mean image*/

    }

    cout << "\nstereo pyramid building done. max levels = " << m_max_levels << endl;
}

void HumanBodyScanner::match()
{
    build_stereo_pyramid();

    for(int p = m_max_levels - 1, cnt = 2; p >= 0; --p, ++cnt) {
        //support window size: 5 -> 7 -> 9 -> 11
        int wnd_sz = 2 * cnt + 1;

        m_stereo_pyramid[p]->set_wnd_size(wnd_sz);
        m_stereo_pyramid[p]->calc_mean_images();
        m_stereo_pyramid[p]->set_patch_params(wnd_sz + 2);
        m_stereo_pyramid[p]->calc_support_region();
        //m_stereo_pyramid[p]->set_voting_params(wnd_sz);


        double duration = (double)getTickCount();
        if(m_max_levels - 1 == p)
            m_stereo_pyramid[p]->calc_init_disparity();
        else
            m_stereo_pyramid[p]->calc_init_disparity(m_stereo_pyramid[p+1]->get_bestk_l(), m_stereo_pyramid[p+1]->get_bestk_r());
        m_stereo_pyramid[p]->calc_seed_disparity();

        printf( "\n\t--------------------------------------------------------\n" );
        printf( "\tseeds selection: %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );   // the elapsed time in sec
        printf( "\t--------------------------------------------------------\n" );
#if DEBUG
        m_debugger->set_data_source(m_stereo_pyramid[p]);                      //set data source
        m_debugger->save_init_infos(p);                                        //save initial disparity
        m_debugger->save_seed_infos(p);                                        //save disparity seeds
#endif

        /*---------------------------------------------------------------------------------------------------------------------*/
        /*                                              propagation                                                            */
        /*---------------------------------------------------------------------------------------------------------------------*/
        printf( "\n\tdisparity seeds propagation ...");
        duration = (double)getTickCount();
        m_stereo_pyramid[p]->disp_2_matches();                                     //disparity to matches
        m_stereo_pyramid[p]->seed_propagate(0);                                    //left->right propagation
        m_stereo_pyramid[p]->seed_propagate(1);                                    //right->left propatation
        m_stereo_pyramid[p]->matches_2_disp();                                     //matches to disparity
        m_stereo_pyramid[p]->cross_validation();                                   //crosss check
        printf( "\n\t--------------------------------------------------------\n" );
        printf( "\texpansion: %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );
        printf( "\t--------------------------------------------------------\n" );
#if DEBUG
        m_debugger->save_prop_infos(p);                                                   //save propagate disparitys
#endif

#if 1
        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                           re-match unexpanded pixels                                               */
        /*--------------------------------------------------------------------------------------------------------------------*/
        //@add by ranqing : can be replaced by qx-unpsampling codes
        printf("\n\trematch unexpanded disparities ...\n");
        duration = (double)getTickCount();
        m_stereo_pyramid[p]->re_match_l();
        m_stereo_pyramid[p]->re_match_r();
        m_stereo_pyramid[p]->cross_validation();
        printf( "\n\t--------------------------------------------------------\n" );
        printf( "\trematch: %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );
        printf( "\t--------------------------------------------------------\n" );
#if DEBUG
        m_debugger->save_rematch_infos(p);
#endif
#endif

#if 1
        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                           upsampling expanded pixels using guidiance filter                        */
        /*--------------------------------------------------------------------------------------------------------------------*/
        //@add by ranqing : 20170119
        printf("\n\tumsampling expanded disparities using guidiance filter ...\n");
        duration = (double)getTickCount();
        m_stereo_pyramid[p]->upsampling_using_rbf();
        m_stereo_pyramid[p]->cross_validation();
        printf( "\n\t--------------------------------------------------------\n" );
        printf( "\tupsampling: %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );
        printf( "\t--------------------------------------------------------\n" );
#if DEBUG
        m_debugger->save_upsamling_infos(p);
#endif
# endif

# if 0
        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                           scanline optimization                                                    */
        /*--------------------------------------------------------------------------------------------------------------------*/
        printf("\n\tscanline optimization ...\n");
        duration = (double)getTickCount();
        m_stereo_pyramid[p]->scanline_optimize(0);
        m_stereo_pyramid[p]->scanline_optimize(1);
        m_stereo_pyramid[p]->cross_validation();
        printf( "\n\t--------------------------------------------------------\n" );
        printf( "\tscanline optimization : %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );
        printf( "\t--------------------------------------------------------\n" );
# if DEBUG
        m_debugger->save_so_infos(p);                                     //save scanline optimize infos
# endif
# endif

        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                         disparity refinement : region voting                                       */
        /*--------------------------------------------------------------------------------------------------------------------*/
        printf("\n\tdisparity general refinment...region voting...%d times\n", REGION_VOTE_TIMES);
        m_stereo_pyramid[p]->init_region_voter();
        for(int t = 0; t < REGION_VOTE_TIMES; ++t) {
            duration = (double)getTickCount();
            m_stereo_pyramid[p]->check_outliers_l();                  //occlusion: 1(Red);   Mismatch: 2(Green)
            m_stereo_pyramid[p]->check_outliers_r();
            m_stereo_pyramid[p]->region_voting();

            printf( "\t--------------------------------------------------------\n" );
            printf( "\tregion voting: %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );
            printf( "\t--------------------------------------------------------\n" );
        }
#if DEBUG
        m_debugger->save_rv_infos(p);                                      //save region voting infos
#endif

        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                         disparity refinement : median filter                                       */
        /*--------------------------------------------------------------------------------------------------------------------*/
        //ranqing @ 2016.11.16
        printf("\n\tdisparity general refinment...median filter...\n");
        duration = (double)getTickCount();
        m_stereo_pyramid[p]->median_filter();
        m_stereo_pyramid[p]->cross_validation();
        printf( "\t--------------------------------------------------------\n" );
        printf( "\tmedian filter : %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency() );
        printf( "\t--------------------------------------------------------\n" );
#if DEBUG
        m_debugger->save_median_infos(p);
#endif
        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                             save results                                                           */
        /*--------------------------------------------------------------------------------------------------------------------*/
        m_debugger->save_final_infos(p);
        printf( "\t--------------------------------------------------------\n" );
        printf( "\tend of stereo matching in level %d\n", p);
        printf( "\t--------------------------------------------------------\n\n" );

#if 0
        /*--------------------------------------------------------------------------------------------------------------------*/
        /*                                             subpixel: interplotaion                                                */
        /*--------------------------------------------------------------------------------------------------------------------*/
        duration = (double)getTickCount();
        m_stereo_pyramid[p]->subpixel_enhancement();
        printf( "\n\t--------------------------------------------------------\n" );
        printf( "\trefined by subpixel interpolation. %.2lf s\n", ((double)(getTickCount())-duration)/getTickFrequency());
        printf( "\t--------------------------------------------------------\n" );
#ifdef  DEBUG
        m_debugger->save_subpixel_infos(p);
#endif
#endif
    }

    //copy disparity results from stereo pyramid
    copy_disp_from_stereo();
}

void HumanBodyScanner::copy_disp_from_stereo() {
    cout << "\n\tcopy disparity from stereo pyramid..." << endl;
    m_dispL = Mat::zeros(m_size, CV_32FC1);
    m_dispR = Mat::zeros(m_size, CV_32FC1);
    m_disp = Mat::zeros(m_size, CV_32FC1);

    vector<float>& disp_vec_l = m_stereo_pyramid[0]->get_disp_l();
    vector<float>& disp_vec_r = m_stereo_pyramid[0]->get_disp_r();
    vector<float>& disp_vec   = m_stereo_pyramid[0]->get_disp();

    m_dispL = Mat::zeros(m_size, CV_32FC1);
    m_dispR = Mat::zeros(m_size, CV_32FC1);
    m_disp  = Mat::zeros(m_size, CV_32FC1);
    qing_vec_2_img<float>(disp_vec_l, m_dispL);
    qing_vec_2_img<float>(disp_vec_r, m_dispR);
    qing_vec_2_img<float>(disp_vec, m_disp);

# if 0
    Mat disp_img_l(m_size, CV_8UC1);
    Mat disp_img_r(m_size, CV_8UC1);
    Mat disp_img(m_size, CV_8UC1);
    float scale = m_stereo_pyramid[0]->get_scale();
    m_dispL.convertTo(disp_img_l, CV_8UC1, scale);
    m_dispR.convertTo(disp_img_r, CV_8UC1, scale );
    m_disp.convertTo(disp_img, CV_8UC1, scale);

    Mat small_disp_l, small_disp_r, small_disp;

    Size dsize = Size(0.25 * m_size.width, 0.25 * m_size.height);
    resize(disp_img_l, small_disp_l, dsize);
    imshow("disp_l", small_disp_l);
    resize(disp_img_r, small_disp_r, dsize);
    imshow("disp_r", small_disp_r);
    resize(disp_img, small_disp, dsize);
    imshow("disp", small_disp);
    waitKey(0);
    destroyAllWindows();
#endif
}

void HumanBodyScanner::disp_2_depth(const Mat &dsp, const Mat &msk, const Mat &img, vector<Vec3f> &points, vector<Vec3f> &colors) {
    float * ptr_dsp = (float *)dsp.ptr<float>(0);
    uchar * ptr_msk = (uchar *)msk.ptr<uchar>(0);
    uchar * ptr_rgb = (uchar *)img.ptr<uchar>(0);

    double * qmtx = (double *)m_qmatrix.ptr<double>(0);

    int w = m_size.width;
    int h = m_size.height;

    points.reserve(w*h);
    colors.reserve(w*h);

    for(int y = 0; y < h; ++y) {
        for(int x = 0; x < w; ++x) {
            int index = y * w + x;
            if( 0 == ptr_msk[index] || 0 == ptr_dsp[index] ) continue;
            else {
                double uvd1[4], xyzw[4] ;
                uvd1[0] = x + m_crop_pointL.x;
                uvd1[1] = y + m_crop_pointL.y;
                uvd1[2] = ptr_dsp[index];
                uvd1[3] = 1.0;

                xyzw[0] = qmtx[ 0] * uvd1[0] + qmtx[ 1] * uvd1[1] + qmtx[ 2] * uvd1[2] + qmtx[ 3] * uvd1[3];
                xyzw[1] = qmtx[ 4] * uvd1[0] + qmtx[ 5] * uvd1[1] + qmtx[ 6] * uvd1[2] + qmtx[ 7] * uvd1[3];
                xyzw[2] = qmtx[ 8] * uvd1[0] + qmtx[ 9] * uvd1[1] + qmtx[10] * uvd1[2] + qmtx[11] * uvd1[3];
                xyzw[3] = qmtx[12] * uvd1[0] + qmtx[13] * uvd1[1] + qmtx[14] * uvd1[2] + qmtx[15] * uvd1[3];

                points.push_back( Vec3f(xyzw[0]/xyzw[3], xyzw[1]/xyzw[3], xyzw[2]/xyzw[3]) );
                colors.push_back( Vec3f(ptr_rgb[3*index + 0], ptr_rgb[3*index + 1], ptr_rgb[3*index + 2]) );
# if 0
                cout << uvd1[0] << ' ' << uvd1[1] << ' ' << uvd1[2] << ' ' << uvd1[3] << endl;
                cout << m_qmatrix << endl;
                cout << xyzw[0] << ' ' << xyzw[1] << ' ' << xyzw[2] << ' ' << xyzw[3] << endl;
                cout << points[points.size() - 1] << "\tgetchar(): " << getchar() << endl;
# endif
            }
        }
    }
}

void HumanBodyScanner::triangulate() {
    cout << "\n triangulate 3d points from disparity results..." << endl;

    int w = m_size.width;
    int h = m_size.height;

    Mat erode_msk  = qing_erode_image(m_mskL, 20);

    float * ptr_disp = (float *)m_disp.ptr<float>(0);
    uchar * ptr_msk  = (uchar *)erode_msk.ptr<uchar>(0);

    //preparing disparity
    float offset = m_crop_pointL.x - m_crop_pointR.x;
    for(int y = 0, index = 0; y < h; ++y) {
        for(int x = 0; x < w; ++x) {
            if( 255 == ptr_msk[index] && 0 != (int)ptr_disp[index] )  ptr_disp[index] += offset;
            else  ptr_disp[index] = 0.f;
            index ++;
        }
    }

    //preparing color
    Mat color_img;
    m_imgL.convertTo(color_img, CV_8UC3, 255.0);

    vector<Vec3f> points(0);
    vector<Vec3f> colors(0);

    disp_2_depth(m_disp, erode_msk, color_img, points, colors);
    string savefn = m_out_dir + "/" + m_frame_name + "_pointcloud_" + m_stereo_name + ".ply";
    qing_write_point_color_ply(savefn, points, colors);
    cout << "\nsave " << savefn << " done. " << points.size() << " Points." << endl;
}
