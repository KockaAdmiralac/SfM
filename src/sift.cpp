#include "sift.hpp"
#include <cmath>
using namespace std;
using namespace cv;
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

Mat findKeyPoints(Mat x, Mat y, Mat z)
{
    Mat result(y.cols,y.rows,0);
    bool flag;
    for(int i=0 ; i < result.rows ; i++)
    {   
        for(int j=0 ; j < result.cols ; j++)
        {
            if(i == 0 || j == 0 || i == result.rows-1 || j == result.cols-1){
                result.at<uint8_t>(i,j) = 0;
                continue;
            }

            uint8_t temp = y.at<uint8_t>(i,j);
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
                    if((x.at<uint8_t>(i+p,j+q) >= temp) || (z.at<uint8_t>(i+p,j+q) >= temp) || (y.at<uint8_t>(i+p,j+q) >= temp))
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


void SIFT(Mat original)
{   
    cout << "========= started SIFT ========" << endl;
    Mat gray_image; 
    cvtColor( original, gray_image, COLOR_BGR2GRAY ); //converting image to grayscale
    int blur_levels = 5; //number of blur_levels, reconsider findng out how to calculate this
    //the blur levels and number of octaves depends on original image size,
    //but the autor of sift suggest that 5 BL and 4 Octaves are the best for SIFT.
    int octaves = 4; //number of octaves 
    double sigma0 = 1.6;
    double current_sigma = sigma0;
    Mat temp;
    Mat ScaleSpace[octaves][blur_levels];

    ScaleSpace[0][0] = gaussianBlur(gray_image,5,current_sigma);

    for(int i =0;i<octaves;i++)
    {
        if( i >0 )
        {
            ScaleSpace[i][0] = downsample(ScaleSpace[i-1][4]);
        }

        for(int j =1; j<blur_levels; j++)
        {
            current_sigma = sigma0 * pow(2.0,i+uint8_t(j)/blur_levels); // according to comment on [1].
            printf("sigma for picture ScaleSpace[%d][%d] is %f\n",i,j,current_sigma);
            ScaleSpace[i][j] = gaussianBlur(ScaleSpace[i][j-1],5,current_sigma);
        }
    }
    // no need for normalization of the images ? 
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
    //LoG approximation:

    Mat DoG[octaves][blur_levels-1];
    
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


    //Finding local minima and maxima:

    Mat keyPointMaps[octaves][blur_levels];
    
    for(int i=0; i<octaves; i++)
    {
        for(int j = 0; j<2 ; j++)
        {
            keyPointMaps[i][j] = findKeyPoints(DoG[i][j],DoG[i][j+1],DoG[i][j+2]);
            
            string str = "TEMP/keyPointMaps/";
            str.append(to_string(i));
            str.append(to_string(j));
            str.append(".PNG");
            imwrite(str,keyPointMaps[i][j]);
        }
    }

}

// [1]: https://www.researchgate.net/post/Can_any_one_help_me_understand_Deeply_SIFT