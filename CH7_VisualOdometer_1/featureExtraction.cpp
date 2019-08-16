#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int main(int argc, char const *argv[]) {
  if (argc != 3) {
    cout << "Usage: featureExtraction image1 image2 " << endl;
    return 1;
  }
  Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

  vector<KeyPoint> keypoint_1, keypoint_2;
  Mat descriptor_1, descriptor_2;
  Ptr<ORB> orb =
      cv::ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);
  orb->detect(img_1, keypoint_1);
  orb->detect(img_2, keypoint_2);

  orb->compute(img_1, keypoint_1, descriptor_1);
  orb->compute(img_2, keypoint_2, descriptor_2);

  Mat output_img1;
  drawKeypoints(img_1, keypoint_1, output_img1, Scalar::all(-1),
                DrawMatchesFlags::DEFAULT);
  imshow("ORB 特征点 ", output_img1);
  vector<DMatch> matches;
  BFMatcher matcher(NORM_HAMMING);
  matcher.match(descriptor_1, descriptor_2, matches);
  double min_dist = 10000, max_dist = 0;
  for (int i = 0; i < descriptor_1.rows; i++) {
    double dist = matches[i].distance;
    if (dist < min_dist) {
      min_dist = dist;
    }
    if (dist > max_dist) {
      max_dist = dist;
    }
  }
  cout << "Max dist = " << max_dist << endl;
  cout << "Min dist = " << min_dist << endl;

  vector<DMatch> good_matches;

  for (int i = 0; i < descriptor_1.rows; i++) {
    if (matches[i].distance <= max(2 * min_dist, 30.0)) {
      good_matches.push_back(matches[i]);
    }
  }

  Mat imgMatch, imgGoodMatch;
  drawMatches(img_1, keypoint_1, img_2, keypoint_2, matches, imgMatch);
  drawMatches(img_1, keypoint_1, img_2, keypoint_2, good_matches, imgGoodMatch);

  cv::namedWindow("All match point ", CV_WINDOW_NORMAL);
  imshow("All match point ", imgMatch);
  cv::namedWindow("Good match point ", CV_WINDOW_NORMAL);
  imshow("Good match point ", imgGoodMatch);
  waitKey(0);
  return 0;
}
