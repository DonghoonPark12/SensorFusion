#ifndef dataStructures_h
#define dataStructures_h

#include <vector>
#include <opencv2/core.hpp>

struct LidarPoint { // single lidar point in space
    double x,y,z,r; // x,y,z in [m], r is point reflectivity
};

struct cmp{
    bool operator()(LidarPoint const &l, LidarPoint const &r){
        return l.y < r.y;
    }
}

#endif /* dataStructures_h */
