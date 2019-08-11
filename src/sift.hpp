#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;
//steps:
    //resize using bi-linear interpolation (don't think we really need to do this)
    //Gaussian convolution

Mat downsample(Mat image);

void SIFT(Mat image);