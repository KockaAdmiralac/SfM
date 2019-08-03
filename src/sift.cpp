#include "sift.hpp"

Mat gaussianBlur(Mat image, int x, double sigma)
{
    Mat temp;
    GaussianBlur(image,temp,Size(x,x),sigma);
    return temp;
}

Mat downsample(Mat image)
{
    Mat temp;
    pyrDown(image,temp,Size(image.cols/2, image.rows/2));
    return temp;
}


void SIFT(Mat original)
{   
    
    


}