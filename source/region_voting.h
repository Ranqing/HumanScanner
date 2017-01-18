#ifndef REGION_VOTING_H
#define REGION_VOTING_H

#define TAUS 15
#define TAUH 0.4f

class RegionVoter
{
public:
     RegionVoter();
    ~RegionVoter();

private:
     int m_taus;
     float m_tauh;

     //region voting
     //    int * m_dist_hist, m_taus;      //histogram of disparites to record votes
     //    float m_tauh;                   //params for region voting
};

inline RegionVoter::RegionVoter(): m_taus(TAUS), m_tauh(TAUH) {

}

inline RegionVoter::~RegionVoter() {

}

#endif // REGION_VOTING_H
