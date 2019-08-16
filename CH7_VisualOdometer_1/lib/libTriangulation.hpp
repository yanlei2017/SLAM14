
#ifndef LIBTRIANGULATION_H
#define LIBTRIANGULATION_H

#include "libfeatureExtraction.hpp"
void triangulation(const vector<KeyPoint> keypoints_1,
                   const vector<KeyPoint> keypoints_2,
                   vector<DMatch> &matches,
                   Mat &R, Mat &t,
                   vector<Point3d> &points);
#endif