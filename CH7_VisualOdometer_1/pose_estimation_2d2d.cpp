#include "lib/libfeatureExtraction.hpp"
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp> //计算基础矩阵...
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

int main(int argc, char const *argv[]) {
  if (argc != 3) {
    cout << "Usage: featureExtraction image1 image2 " << endl;
    return 1;
  }
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
  vector<KeyPoint> keypoints_1, keypoints_2;
  vector<DMatch> matches;
  findFeatureMatchs(img_1, img_2, keypoints_1, keypoints_2, matches);
  cout << matches.size() << " match point found !" << endl;
  Mat R, t;
  pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);
  //验证E=t^R*scale
  Mat t_x = (Mat_<double>(3, 3) << 0, -t.at<double>(2, 0), t.at<double>(1, 0),
             t.at<double>(2, 0), 0, -t.at<double>(0, 0), -t.at<double>(1, 0),
             t.at<double>(0, 0), 0);
  cout << "t^R= " << endl << t_x * R<<endl;
  //验证对极约束
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  for (DMatch i : matches) {
    Point2d pt1 = pixel2cam(keypoints_1[i.queryIdx].pt, K);
    Mat y1 = (Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
    Point2d pt2 = pixel2cam(keypoints_2[i.trainIdx].pt, K);
    Mat y2 = (Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
    Mat d = y2.t() * t_x * R * y1;
    cout << "Epipolar constraint : " << d << endl;
  }
  return 0;
}