#include "lib/libTriangulation.hpp"
int main(int argc, char const *argv[])
{
    if (argc != 3)
    {
        cout << "Usage: featureExtraction image1 image2 " << endl;
        return 1;
    }
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    findFeatureMatchs(img_1, img_2, keypoints_1, keypoints_2, matches);
    Mat R, t;
    pose_estimation_2d2d(keypoints_1, keypoints_2, matches, R, t);

    vector<Point3d> points;
    triangulation(keypoints_1, keypoints_2, matches, R, t, points); //三角化后求得的点在points
    Mat K = (Mat_<double>(3, 3) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1);
    //验证三角化点与特征点的重投影关系
    for (int i = 0; i < matches.size(); i++)
    {
        Point2d pt1_cam = pixel2cam(keypoints_1[matches[i].queryIdx].pt, K);
        Point2d pt1_cam_3d(points[i].x / points[i].z,
                           points[i].y / points[i].z);
        cout << "Point int the first camera frame: " << pt1_cam << endl;
        cout << "Point projected from 3D :" << pt1_cam_3d << ", d =" << points[i].z << endl;
        //第二幅图
        Point2f pt2_cam = pixel2cam(keypoints_2[matches[i].trainIdx].pt, K);
        Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        pt2_trans /= pt2_trans.at<double>(2, 0);
        cout << "Point int the second camera frame: " << pt2_cam << endl;
        cout << "Point projected from second :" << pt2_trans.t() << endl<<endl;
    }

    return 0;
}
