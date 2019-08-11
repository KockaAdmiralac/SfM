#include "kanatani.hpp"

cv::Mat S;
/*
VertexList kanatani(std::vector<cv::Mat> x, std::vector<cv::Mat> x1, cv::Mat E) {
    S.at<int>(0, 0) = 1;
    S.at<int>(0, 1) = 0;
    S.at<int>(0, 2) = 0;
    S.at<int>(1, 0) = 0;
    S.at<int>(1, 1) = 1;
    S.at<int>(1, 2) = 0;
    std::vector<cv::Mat> dx1;
    std::vector<cv::Mat> dx2;
    std::vector<cv::Mat> n;
    std::vector<cv::Mat> n1;
    cv::Mat En = S * E * S.t();
    for (int k = 0; k < x.size(); ++k) {
        n.push_back(S * E * x1[k - 1]);
        n1.push_back(S * E.t() * x[k - 1]);
        cv::Mat ak = n[k].t() * En * n1[k];
        cv::Mat bk = 0.5 * (n[0].t() * n[k] + n1[0].t() * n1[k]);
        cv::Mat ck = x[0].t() * E * x1[0];
        // ???
    }
}
*/
