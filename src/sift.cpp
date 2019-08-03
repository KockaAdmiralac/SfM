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
            current_sigma = sigma0 * pow(2.0,i+double(j)/blur_levels); // according to comment on [1].
            printf("sigma for picture ScaleSpace[%d][%d] is %f\n",i,j,current_sigma);
            ScaleSpace[i][j] = gaussianBlur(ScaleSpace[i][j-1],5,current_sigma);
        }
    }

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

}

// [1]: https://www.researchgate.net/post/Can_any_one_help_me_understand_Deeply_SIFT