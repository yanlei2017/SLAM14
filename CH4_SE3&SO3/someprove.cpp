#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <sophus/so3.h>
#include <sophus/se3.h>
using namespace std;
int main(int argc, char const *argv[])
{
    Eigen::Vector3d so3(sqrt(2) / 2, sqrt(2) / 2, 0);
    cout << "Vector3d so3 =\n"
         << so3.transpose() << endl;
    Eigen::Matrix3d SO3 = Sophus::SO3::hat(so3);
    cout << "Matrix3d SO3 =\n"
         << SO3 << endl;
    Eigen::Matrix3d RES = Eigen::Matrix3d::Zero();
    RES = SO3 * SO3 + Eigen::Matrix3d::Identity() - so3 * so3.transpose(); //prove for page69 - 4.20
    cout << "RES =SO3 * SO3 + I - so3 * so3 =\n" << RES << endl
         << endl;
    cout << "SO3 * SO3 * SO3 + SO3 = \n"<<SO3 * SO3 * SO3 + SO3 << endl; //prove for page69 - 4.21
    return 0;
}
