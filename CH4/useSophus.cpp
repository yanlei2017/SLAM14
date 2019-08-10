#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <sophus/so3.h>
#include <sophus/se3.h>

using namespace std;

int main(int argc, char const *argv[])
{
    cout.precision(4);
    cout.width(8);
    cout.setf(ios::right);
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_V(0, 0, M_PI_2);
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);
    cout << "SO(3) from matrix = \n"
         << SO3_R.matrix() << endl; //if without .matrix(), the output would be so3
    cout << "SO(3) from vector = \n"
         << SO3_V.matrix() << endl;
    cout << "SO(3) from Quanternion = \n"
         << SO3_q.matrix() << endl;

    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so3 = \n"
         << so3.transpose() << endl;
    cout << "so3^ = \n"
         << Sophus::SO3::hat(so3) << endl; //transform vector3d to inverse matrix3d
    cout << "so3^ vee = \n"
         << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl; //transform inverse matrix3d  to vector3d

    Eigen::Vector3d update_so3(3, 0, 0);
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R; //left multiply to update rotation matrix
    cout << "SO3 updated = \n"
         << SO3_updated << endl;

    /************************SE3***************************/
    Eigen::Vector3d t(1, 0, 0);
    Sophus::SE3 SE3_Rt(R, t);
    Sophus::SE3 SE3_qt(q, t);
    cout << "SE(3) from matrix and t = \n"
         << SE3_Rt.matrix() << endl;
    cout << "SE(3) from Quanternion and t = \n"
         << SE3_qt.matrix() << endl;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d se3 = SE3_Rt.log();
    cout << "se3 of SE3_Rt = \n"
         << se3.transpose() << endl;
    cout << "se3^ =\n"
         << Sophus::SE3::hat(se3) << endl;
    cout << "see^ vee = \n"
         << Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose() << endl;
    Vector6d update_se3;
    update_se3.setZero();
    update_se3(0, 0) = 1e-4;
    Sophus::SE3 SE3_updated = Sophus::SE3::exp(update_se3) * SE3_Rt;
    cout << "SE3 updated = \n"
         << SE3_updated.matrix() << endl;

    return 0;
}
