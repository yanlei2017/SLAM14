#include "libfeatureExtraction.hpp"
void pose_estimation_2d2d(const vector<KeyPoint> &keypoints_1,
                          const vector<KeyPoint> &keypoints_2,
                          const vector<DMatch> &matches, Mat &R, Mat &t)
{
  Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
  vector<Point2f> points1;
  vector<Point2f> points2;

  for (int i = 0; i < matches.size(); i++)
  {
    points1.push_back(keypoints_1.at(matches[i].queryIdx).pt); //queryIdx和trainIdx都是关键点索引，见68行代码
    points2.push_back(keypoints_2.at(matches[i].trainIdx).pt);
  }
  //计算基础矩阵 fundamental_matrix

  Mat fundamental_matrix;
  fundamental_matrix = findFundamentalMat(points1, points2, CV_FM_8POINT);
  cout << "Fundamental matrix is " << endl
       << fundamental_matrix << endl;

  //计算本质矩阵 essential
  Point2d principal_point(325.1, 249.7); //光心 TUM dataset 标定值
  double focal_length = 521.0;           //焦距 TUM dataset 标定值
  Mat essential_matrix;
  essential_matrix =
      // findEssentialMat(points1, points2, focal_length, principal_point, RANSAC);
      findEssentialMat(points1, points2, focal_length, principal_point);
  cout << "Essential matrix is \n"
       << essential_matrix << endl;

  //计算单应矩阵 H
  Mat homography_matrix;
  homography_matrix =
      findHomography(points1, points2, RANSAC, 3, noArray(), 2000, 0.99); //RANSAC最多迭代2000次
                                                                          //https://blog.csdn.net/fengyeer20120/article/details/87798638
  cout << "Homography matrix is \n"
       << homography_matrix << endl;
  recoverPose(essential_matrix, points1, points2, R, t, focal_length,
              principal_point);
  cout << "R is \n"
       << R << endl;
  cout << "t is \n"
       << t << endl;
}

void findFeatureMatchs(const Mat &img_1, const Mat &img_2,
                       vector<KeyPoint> &keypoints_1,
                       vector<KeyPoint> &keypoints_2,
                       vector<DMatch> &good_matches)
{
  Mat descriptor_1, descriptor_2;

  // Ptr<ORB> orb =
  //     cv::ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
  Ptr<FeatureDetector> orb = ORB::create();
  Ptr<DescriptorExtractor> descriptor = ORB::create();
  Ptr<DescriptorMatcher> matcher =
      DescriptorMatcher::create("BruteForce-Hamming");

  orb->detect(img_1, keypoints_1);
  orb->detect(img_2, keypoints_2);

  orb->compute(img_1, keypoints_1, descriptor_1);
  orb->compute(img_2, keypoints_2, descriptor_2);

  // BFMatcher matcher(NORM_HAMMING);
  vector<DMatch> matches;
  matcher->match(descriptor_1, descriptor_2, matches);
  //matches[1].queryIdx是descriptor_1的关键点的索引;
  //matches[1].trainIdx是descriptor_2的关键点的索引;

  double min_dist = 10000, max_dist = 0;
  for (int i = 0; i < descriptor_1.rows; i++)
  {
    double dist = matches[i].distance;
    if (dist < min_dist)
    {
      min_dist = dist;
    }
    if (dist > max_dist)
    {
      max_dist = dist;
    }
  }
  cout << "Max dist = " << max_dist << endl;
  cout << "Min dist = " << min_dist << endl;

  for (int i = 0; i < descriptor_1.rows; i++)
  {
    if (matches[i].distance <= max(2 * min_dist, 30.0))
    {
      good_matches.push_back(matches[i]);
    }
  }
}

Point2d pixel2cam(const Point2d &p, const Mat &K)
{
  return Point2d((p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),  //像素(图片)坐标x--c_x/f_x
                 (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)); //像素(图片)坐标y-c_y/f_y
}