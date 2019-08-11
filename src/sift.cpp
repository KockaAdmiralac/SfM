#include "sift.hpp"
#include <cmath>

cv::Mat gaussianBlur(cv::Mat image, int x, double sigma)
{
    cv::Mat temp;
    GaussianBlur(image,temp,Size(x,x),sigma);
    return temp;
}

cv::Mat downsample(cv::Mat image)
{
    cv::Mat temp;
    pyrDown(image,temp,Size(image.cols/2, image.rows/2));
    return temp;
}


/**
 * @brief Find Keypoints on a single image.
 * @param x imageBelow image that is below (w.r.t. Scale Space order of one octave) the imageTarget
 * @param y imageTarget image that is target for finding keypoints
 * @param z imageAbove image that is above (w.r.t. Scale Space order of one octave) the imageTarget
 */
cv::Mat findKeyPoints(cv::Mat imageBelow, cv::Mat imageTarget, cv::Mat z)
{
    cv::Mat result;
    bool flag;
    for(int i=0 ; i < result.rows ; i++)
    {   
        for(int j=0 ; j < result.cols ; j++)
        {
            if(i == 0 || j == 0 || i == result.rows-1 || j == result.cols-1){
                result.at<uint8_t>(i,j) = 0;
                continue;
            }

            uint8_t temp = imageTarget.at<uint8_t>(i,j);
            flag = true;
            /* 
            for(int p = -1; p<=1;p++)
            {
                /out <<" | ";
                for(int q = -1; q<= 1; q++)
                {
                    cout << unsigned(x.at<uint8_t>(i+p,j+q))<<", ";
                }
                cout <<" | " << endl;
            }
            */

            for(int p = -1; p<=1;p++)
            {
                for(int q = -1; q<= 1; q++)
                {
                    if(p == 0 && q == 0) continue;
                    if((imageBelow.at<uint8_t>(i+p,j+q) >= temp) || (z.at<uint8_t>(i+p,j+q) >= temp) || (imageTarget.at<uint8_t>(i+p,j+q) >= temp))
                    {
                        result.at<uint8_t>(i,j) = 0;
                        flag = false;
                    }
                }
            }
            if(flag)
            {
                result.at<uint8_t>(i,j) =255;
            }

        }

    }
    return result;
}


/**
 * @brief Executed on every frame, to find Keypoints and calculate their descriptors.
 * The number of blur levels (images per octave) is 5.
 * The number of octaves is 4. 
 * Both numbers (num of images per octave, and num of octaves) are suggested by Dr. Lowe.
 * @param original Image on which SIFT is going to be applied.
 * @param keypointsContainer contiainer for Keypoints, pass empty std::vector<cv::Keypoints>
 * @param descriptors container for descriptors, pass cv::Mat
 */
void SIFT(cv::Mat original,std::vector<cv::KeyPoint> &keypointsContainer,cv::Mat &descriptors)
{   
    cout << "========= started SIFT ========" << endl;
    cv::Mat gray_image; 
    cv::cvtColor( original, gray_image, COLOR_BGR2GRAY ); //converting image to grayscale
    int blur_levels = 5;
    int octaves = 4;
    double sigma0 = 1.6; //starting sigma
    double current_sigma = sigma0; //temp for sigma value according to what image on scale space is being generated
    cv::Mat ScaleSpace[octaves][blur_levels];

    ScaleSpace[0][0] = gaussianBlur(gray_image,5,current_sigma);

    for(int i =0;i<octaves;i++)
    {
        if( i >0 )
            ScaleSpace[i][0] = downsample(ScaleSpace[i-1][4]); //if shifted to a new octave, scale down the image

        for(int j =1; j<blur_levels; j++)
        {
            current_sigma = sigma0 * pow(2.0,i+uint8_t(j)/blur_levels); // according to comment on [1].
            ScaleSpace[i][j] = gaussianBlur(ScaleSpace[i][j-1],5,current_sigma);
        }
    }
    // no need for normalization of the images ?
    #ifdef DEBUG_MODE
    for(int i=0;i<octaves;i++)
    {
        for(int j =0;j<blur_levels;j++)
        {
            string str = "TEMP/ScaleSpaceTemp/";
            str.append(to_string(i));
            str.append(to_string(j));
            str.append(".JPG");
            imwrite(str,ScaleSpace[i][j]);
        }
    }
    #endif
    //LoG approximation:

    cv::Mat DoG[octaves][blur_levels-1];
    
    #ifdef DEBUG_MODE
    for(int i =0;i<octaves;i++)
    {
        for(int j =0;j<blur_levels-1;j++)
        {
            subtract(ScaleSpace[i][j],ScaleSpace[i][j+1],DoG[i][j]);
            string str = "TEMP/DoGTemp/";
            str.append(to_string(i));
            str.append(to_string(j));
            str.append(".PNG");
            imwrite(str,DoG[i][j]);
        }
    }
    #endif

    //Finding local minima and maxima:

    cv::Mat keyPointMaps[octaves][blur_levels];
    
    for(int i=0; i<octaves; i++)
    {
        for(int j = 0; j<2 ; j++)
        {
            keyPointMaps[i][j] = findKeyPoints(DoG[i][j],DoG[i][j+1],DoG[i][j+2]);
            #ifdef DEBUG_MODE
                string str = "TEMP/keyPointMaps/";
                str.append(to_string(i));
                str.append(to_string(j));
                str.append(".PNG");
                imwrite(str,keyPointMaps[i][j]);
            #endif
        }
    }



}

// [1]: https://www.researchgate.net/post/Can_any_one_help_me_understand_Deeply_SIFT