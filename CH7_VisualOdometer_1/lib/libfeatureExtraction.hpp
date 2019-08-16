#ifndef LIBFEATUREEXTRACTION_H
#define LIBFEATUREEXTRACTION_H


#include <iostream>
#include <opencv2/calib3d/calib3d.hpp> //计算基础矩阵...
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;
void pose_estimation_2d2d(const vector<KeyPoint> &keypoints_1,
                          const vector<KeyPoint> &keypoints_2,
                          const vector<DMatch> &matches, Mat &R, Mat &t);

void findFeatureMatchs(const Mat &img_1, const Mat &img_2,
                       vector<KeyPoint> &keypoints_1,
                       vector<KeyPoint> &keypoints_2,
                       vector<DMatch> &good_matches);

Point2d pixel2cam(const Point2d &p, const Mat &K);
#endif