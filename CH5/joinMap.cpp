#include<iostream>
#include<fstream>
using namespace std;
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<eigen3/Eigen/Geometry>

#include<boost/format.hpp>

#include<pcl-1.8/pcl/point_types.h>
#include<pcl-1.8/pcl/io/pcd_io.h>
#include<pcl-1.8/pcl/visualization/pcl_visualizer.h>


int main(int argc, char const *argv[])
{
    vector<cv::Mat> colorImgs,depthImgs;
    vector<Eigen::Isometry3d,Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    

    return 0;
}
