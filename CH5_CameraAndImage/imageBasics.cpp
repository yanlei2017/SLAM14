#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;

int main(int argc, char const *argv[])
{
    cv::Mat image;
    image = cv::imread(argv[1]);
    if (image.data == nullptr)
    {
        cerr << "File " << argv[1] << "not exits" << endl;
        return 0;
    }
    cout << "With of image = " << image.cols << ",Height of image = " << image.rows << ", Channels of image = " << image.channels() << endl;
    cv::imshow(argv[1], image);
    cv::waitKey(0);
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3)
    {
        cout << "Please input a colorful image. \n";
        return 0;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for (size_t i = 0; i < image.rows; i++)
    {
        for (size_t j = 0; j < image.cols; j++)
        {
            unsigned char *row_ptr = image.ptr<unsigned char>(i);
            unsigned char *data_ptr = &row_ptr[j * image.channels()];
            for (size_t k = 0; k < image.channels(); k++)
            {
                unsigned char data = data_ptr[k];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "Time used for traversing " << argv[1] << " = " << time_used.count() << "seconds." << endl;

    cv::Mat image_another = image;
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);

    cv::imshow("cv::Mat image_another = image;image_another(cv::Rect(0, 0, 100, 100)).setTo(0);", image_another);
    cv::waitKey(0);
    cv::imshow("image", image);
    cv::waitKey(0);

    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);
    cv::imshow(argv[1], image);
    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
