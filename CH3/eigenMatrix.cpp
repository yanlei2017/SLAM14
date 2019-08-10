#include <iostream>
#include <ctime>
using namespace std;
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define MATRIX_SIZE 50
int main(int argc, char const *argv[])
{
    Eigen::Matrix<float, 2, 3> matrix_23f;
    Eigen::Vector3d v_3d;
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> maitix_dynamic;
    Eigen::MatrixXd matrix_x;

    matrix_23f << 1, 2, 3, 4, 5, 6;
    cout << matrix_23f << endl;
    for (int i = 0; i < 1; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            cout << matrix_23f(i, j) << endl;
        }
    }

    v_3d << 3, 2, 1;

    // Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23f * v_3d;                     //float matrix cant multiply with double matrix
    Eigen::Matrix<double, 2, 1> result = matrix_23f.cast<double>() * v_3d; // tramsform float matrix to double
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23f.cast<double>() * v_3d; // wrong dimension of result matrix
    cout << "\nmatrix_23f * v_3d = \n"
         << result << endl;

    matrix_33 = Eigen::Matrix3d::Random();
    cout << "\nmatrix_33 :\n"
         << matrix_33 << endl;
    cout << "\nmatrix_33 transpose :\n"
         << matrix_33.transpose() << endl; //转置
    cout << "\nmatrix_33 sum :\n"
         << matrix_33.sum() << endl; //各元素和
    cout << "\nmatrix_33 trace :\n"
         << matrix_33.trace() << endl; //迹
    cout << "\nmatrix_33 inverse :\n"
         << matrix_33.inverse() << endl; //逆
    cout << "\nmatrix_33 determinant :\n"
         << matrix_33.determinant() << endl; //行列式
    cout << "\n10 * matrix_33 :\n"
         << 10 * matrix_33 << endl;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "\nEigen values = \n"
         << eigen_solver.eigenvalues() << endl;
    cout << "\nEigen vectors = \n"
         << eigen_solver.eigenvectors() << endl;

    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);
    clock_t time_start_inverse = clock(); //start timer
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "\nTime use in normal inverse is " << 1000 * (clock() - time_start_inverse) / (double)CLOCKS_PER_SEC << "ms" << endl;
    clock_t time_start_qr_composition = clock(); //start timer
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "\nTime use in QR composition is " << 1000 * (clock() - time_start_qr_composition) / (double)CLOCKS_PER_SEC << "ms" << endl;

    return 0;
}
