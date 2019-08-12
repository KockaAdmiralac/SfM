#ifndef RANSAC_HPP
#define RANSAC_HPP

#include <iostream>
#include <vector>
#include <opencv2/ximgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <kitti.hpp>

class ourRANSAC
{
public:
    Sequence *seq;
    int k; //number of iterations
    size_t N; //points per subsequence
    int treshold;
    //params for solvePNP:
    cv::Mat TriangulatedPointsAll;
    std::vector<cv::Point2d> KeyPointsAll,KeyPointsForEval;
    cv::Mat camMatrix;
    std::vector<double> DistortionCoefs;
    cv::Mat rotationVector,finalRotationMatrix;
    cv::Mat translationVector,finalTranslationVector;
    cv::Mat image0,image2;

    cv::Mat TriangulatedPointsSubset;
    std::vector<cv::Point2d> KeypointsSubset;

    ourRANSAC(int k, int N, int Treshold,Sequence *seq);
    void setRANSACParams(int k, int N, int treshold);
    void setRANSACArguments(cv::Mat trp, std::vector<cv::Point2d> skp,std::vector<cv::Point2d> skp0, cv::Mat camMatrix,
                            std::vector<double> DistortionCoefs, cv::Mat rotationVector, cv::Mat translationVector);
    
    void calculateExtrinsics();
    void setImages(cv::Mat a, cv::Mat b);
    void returnValues(cv::Mat &rotationContainer, cv::Mat &translationContainer);


};




#endif