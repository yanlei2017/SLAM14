#include <iostream>
#include <cmath>
using namespace std;
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

int main(int argc, char const *argv[])
{
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
    cout.precision(3);
    cout<<"\ntransform rotation vector( M_PI / 4, Eigen::Vector3d(0, 0, 1) ) to rotation matrix :\n"<<rotation_vector.matrix()<<endl;
    rotation_matrix=rotation_vector.toRotationMatrix();
    Eigen::Vector3d v(1,0,0);
    Eigen::Vector3d v_rotated =rotation_vector*v;
    cout<<"\n(1 0 0) aftre rotation by rotation_vector :\n"<<v_rotated.transpose()<<endl;
    v_rotated=rotation_matrix*v;
    cout<<"\n(1 0 0) aftre rotation by rotation_matrix:\n"<<v_rotated.transpose()<<endl;

    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2,1,0);//(2,1,0 means Z-Y-X rotation order , get yaw pitch roll )
    cout<<"\nyaw pitch roll = "<<euler_angles.transpose()<<endl;

    Eigen::Isometry3d  T=Eigen::Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Eigen::Vector3d(1,3,4));
    cout<<"\nTransform matrix = \n"<<T.matrix()<<endl;

    Eigen::Vector3d v_transformed = T*v;
    cout<<"\nv transformed = "<<v_transformed.transpose()<<endl;

    Eigen::Quaterniond q=Eigen::Quaterniond(rotation_vector);
    cout<<"\nQuaternion of rotation_vector =\n"<<q.coeffs()<<endl;
    q=Eigen::Quaterniond(rotation_matrix);
    cout<<"\nQuaternion of rotation_matrix=\n"<<q.coeffs()<<endl;
    v_rotated=q*v;
    cout<<"\n(1 0 0) after rotation = \n"<<v_rotated.transpose()<<endl;


    return 0;
}
