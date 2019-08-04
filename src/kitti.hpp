#include <opencv2/core/core.hpp>

#include <iterator>

struct Sequence {
    int number;
    std::vector<cv::Mat> poses;
    std::vector<double> times;
    std::vector<cv::Mat> calib;
    cv::Mat tr;
    Sequence(int number);
    cv::Mat image(int image, int sequence);
};
