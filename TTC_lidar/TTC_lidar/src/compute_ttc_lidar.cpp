#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>


#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double &TTC)
{
    // auxiliary variables
    double dT = 0.1;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // 
    sort(lidarPointsPrev.begin(), lidarPointsPrev.end(), cmp); //sort ny y value
    int size_Prev = lidarPointsPrev.size();
    double middle_y_Prev = lidarPointsPrev[size_Prev/2].y;     //pick middle idx point

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (it->y < middle_y_Prev-2 || it->y > middle_y_Prev +2 ) continue;

        minXPrev = minXPrev > it->x ? it->x : minXPrev; //d0
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (it->y < middle_y_Prev-2 || it->y > middle_y_Prev +2 ) continue;
        
        minXCurr = minXCurr > it->x ? it->x : minXCurr; //d1
    }

    // if(minXPrev < minXCurr)
    //     computeTTCLidar(lidarPointsPrev, lidarPointsCurr, TTC)

    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

int main()
{

    std::vector<LidarPoint> currLidarPts, prevLidarPts;
    readLidarPts("../dat/C22A5_currLidarPts.dat", currLidarPts);
    readLidarPts("../dat/C22A5_prevLidarPts.dat", prevLidarPts);


    double ttc;
    computeTTCLidar(prevLidarPts, currLidarPts, ttc);
    cout << "ttc = " << ttc << "s" << endl;
}