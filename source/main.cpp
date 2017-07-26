#include "scanner.h"

int main(int argc, char * argv[])
{
    cout << "usage: " << argv[0]
         << "\t/Users/Qing/Data/20161224/Humans_frame/"
         << "\t/Users/Qing/Data/20161224/Humans_mask/"
         << "\t/Users/Qing/Data/20161224/Humans_stereo/"
         << "\tFRM_0176\tstereo_A01A02.info" << endl;           //stereo

    if(argc != 6)
    {
        cerr << "invalid arguments.." << endl;
        return -1;
    }
    string img_folder  = argv[1];
    string msk_folder  = argv[2] ;
    string stereo_folder = argv[3];
    string frame_name = argv[4];
    string stereo_fn  = argv[5];

    HumanBodyScanner * scanner = new HumanBodyScanner(img_folder, msk_folder, stereo_folder, frame_name, stereo_fn);
    if(scanner->init())
    {
        double duration = (double)(cv::getTickCount());

        scanner->match();
        scanner->triangulate_range_grid();

        duration = (double)(cv::getTickCount()) - duration;
        duration /= cv::getTickFrequency();                       //the elapsed time in sec
        printf( "\n--------------------------------------------------------\n" );
        printf( "Total Time: %.2lf s\n", duration );
        printf( "--------------------------------------------------------\n" );
    }
    return 1;
}
