#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <fstream>
#include <iostream>
using namespace std;
#include <boost/format.hpp>

#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/visualization/pcl_visualizer.h>

int main(int argc, char const *argv[]) {
  vector<cv::Mat> colorImags, depthImags;
  vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d> > poses;
  ifstream fin("/home/yanlei/SLAM14/CH5/file/poses.txt");
  if (!fin) {
    cerr << "Please run in dirs that included poses.txt" << endl;
    return 1;
  }

  for (int i = 0; i < 5; i++) {
    boost::format fmt("/home/yanlei/SLAM14/CH5/file/%s/%d.%s");
    colorImags.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
    depthImags.push_back(
        cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));
    double data[7] = {0};
    for (auto &i : data) {
      fin >> i;
    }
    Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
    Eigen::Isometry3d T(q);
    T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
    poses.push_back(T);
  }

  // inner para for camera
  double cx = 325.5;
  double cy = 253.5;
  double fx = 518.0;
  double fy = 519.0;
  double depthscale = 1000.0;
  cout << "Translating image to pointcloud" << endl;

  typedef pcl::PointXYZRGB PointT;
  typedef pcl::PointCloud<PointT> PointCloud;
  PointCloud::Ptr pointCloud(new PointCloud);
  for (int i = 0; i < 5; i++) {
    cout << "Translating " << i + 1 << endl;
    cv::Mat color = colorImags[i];
    cv::Mat depth = depthImags[i];
    Eigen::Isometry3d T = poses[i];
    for (int x = 0; x < color.rows; x++) {
      for (int y = 0; y < color.cols; y++) {
        unsigned int d = depth.ptr<unsigned short>(x)[y];
        if (d == 0) {
          continue;
        }
        Eigen::Vector3d point;
        point[2] = double(d) / depthscale;
        point[0] = (y - cx) * point[2] / fx;
        point[1] = (x - cy) * point[2] / fx;
        Eigen::Vector3d pointworld = T * point;
        PointT p;
        p.x = pointworld[0];
        p.y = pointworld[1];
        p.z = pointworld[2];
        p.b = color.data[x * color.step + y * color.channels()];
        p.g = color.data[x * color.step + y * color.channels() + 1];
        p.r = color.data[x * color.step + y * color.channels() + 2];
        pointCloud->points.push_back(p);
      }
    }
  }
  pointCloud->is_dense = false;
  cout << "pointCloud has " << pointCloud->size() << " points" << endl;
  pcl::io::savePCDFileBinary("map.pcd", *pointCloud);
  return 0;
}
