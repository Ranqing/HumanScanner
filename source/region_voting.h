#ifndef REGION_VOTING_H
#define REGION_VOTING_H

#include <iostream>
#include <vector>
using namespace std;

#define TAUS 5
#define TAUH 0.4f

class RegionVoter
{
public:
     RegionVoter();
    ~RegionVoter();

     int get_vote_count_thresh()   { return m_taus; }
     float get_vote_ratio_thresh() { return m_tauh; }

private:
     int m_taus;                                    //区域内最少票数
     //vector<int> m_dist_hist;                       //histogram of disparites to record votes
     float m_tauh;                                  //最高票数最少占比
};

inline RegionVoter::RegionVoter():m_taus(TAUS), m_tauh(TAUH) {
    cout << "\tinitialization of region voter..." << endl;

}

inline RegionVoter::~RegionVoter() {

}

#endif // REGION_VOTING_H
