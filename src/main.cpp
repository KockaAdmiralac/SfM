#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
//#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>

#include "klt.hpp"
#include "kanatani.hpp"
#include "sift.hpp"
#include "surf.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main() {    
    Mat imageLeft;
    Mat imageRight;
    string pathLeft = "../kitti/stereoData/00/image_0/"; //path to dataset from current location
    string pathRight = "../kitti/stereoData/00/image_1/";
    int i = 0;
    string full_path_left,full_path_right;
    string extra_path;
    //the main loop
    while(1) //loop while there are pictures example name: PiazzaBra - 000007.JPG
    {
        // loop through images:
        i++;

        if(i<10)            extra_path = "00000";
        else if(i<100)      extra_path = "0000";
        else if(i<1000)     extra_path = "000";
        else if(i<10000)    extra_path = "00";
        else if(i<100000)   extra_path = "0";
        else                extra_path = "";

        extra_path.append(to_string(i));

        full_path_left = pathLeft;
        full_path_left.append(extra_path);
        full_path_left.append(".png");

        full_path_right = pathRight;
        full_path_right.append(extra_path);
        full_path_right.append(".png");
        
        //cout <<"opening " << full_path << endl; 
        try
        {
            imageLeft = imread(full_path_left, IMREAD_COLOR); //already grayscale?
            if(! imageLeft.data)
            {
                cout << "error loading: " << full_path_left << "\n";
                throw -1;
            }

            imageRight = imread(full_path_right, IMREAD_GRAYSCALE);
            if(! imageRight.data)
            {
                cout << "error loading: " << full_path_right << "\n";
                throw -1;
            }
        }

        catch(int n)
        {
            if(n == -1)
            {
                cout <<"breaking out of the main loop.\n";
                break;
            }
        }

    int minHessian = 400;
    Ptr<SURF> detector = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( imageLeft, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( imageRight, noArray(), keypoints2, descriptors2 );

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<DMatch> > knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches,  2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches;
    cout <<"there are total of " << knn_matches.size() << " matches ";
    for (size_t i = 0; i < knn_matches.size(); i++) //knn_matches.size()
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    cout << " and " << good_matches.size() << "good matches \n";
    //-- Draw matches
    int loops = good_matches.size()/20;
    for(int i = 1;i < loops;i++ )
    {
        Mat img_matches;
        std::vector<DMatch>::iterator first = good_matches.begin(); //20*(i-1)
        std::vector<DMatch>::iterator last = good_matches.begin()+i;
        std::vector<DMatch> newVec(first,last);
        cout << "features1: " << newVec.at(i-1).queryIdx <<"with pos: " << descriptors1.at<_Float32>(newVec.at(i-1).queryIdx) << "\n";
        cout << "features2: " << newVec.at(i-1).trainIdx <<"with pos: " << descriptors2.at<_Float32>(newVec.at(i-1).queryIdx) << "\n";
        
        cout << "type = " << descriptors1.type() << "\n";
        
        drawMatches( imageLeft, keypoints1, imageRight, keypoints2, newVec, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        imshow("Good Matches", img_matches );
        waitKey(); 
    }
        
        
        //-- Show detected (drawn) keypoints
        //imshow("Keypoints 1", img_keypoints_1 );
        //imshow("Keypoints 2", img_keypoints_2 );

        //SIFT(image);

        //namedWindow("Display window", WINDOW_AUTOSIZE);
        //imshow("Display window", image);
        //waitKey(0);
        


        //TODO:
            //SIFT --------
                    //test:
                    //Mat temp = downsample(image);
                    //namedWindow("2",WINDOW_AUTOSIZE);
                    //imshow("2",downsample(temp));
                    //waitKey(0);

                //KANADE-LUKAS
                    //kantani triangulation
                        //bundle adj.

            //SURF
                //KANADE-LUKAS
                    //kantani triangulation
                        //bundle adj.

            //KLT
                //katani triangulation
                    //bundle adj.

    }

    return 0;
}


/*
code used for cropping:   
Rect roi;
        if(image.cols == 3008 && image.rows == 2000) {//vertikalno
            roi.x = 4;
            roi.y = 0;
            roi.width = 2000;
            roi.height = 2000;
            Mat cropped1 = image(roi);
            
            roi.x = 1004;
            roi.y = 0;
            roi.width=2000;
            roi.height=2000;
            Mat cropped2 = image(roi);
            
            String str = "../data_SfM/PiazzaBraNewCropped/";
            str.append(to_string(count));
            str.append(".JPG");
            imwrite(str,cropped1);
            count++;

            str = "../data_SfM/PiazzaBraNewCropped/";
            str.append(to_string(count));
            str.append(".JPG");
            imwrite(str,cropped2);
            count++;
        }

        else if(image.cols == 2000 && image.rows == 3008){ //horizontalno
            roi.x = 0;
            roi.y = 4;
            roi.width = 2000;
            roi.height = 2000;
            Mat cropped1 = image(roi);
            
            roi.x = 0;
            roi.y = 1004;
            roi.width=2000;
            roi.height=2000;
            Mat cropped2 = image(roi);
            
            String str = "../data_SfM/PiazzaBraNewCropped/";
            str.append(to_string(count));
            str.append(".JPG");
            imwrite(str,cropped1);
            count++;

            str = "../data_SfM/PiazzaBraNewCropped/";
            str.append(to_string(count));
            str.append(".JPG");
            imwrite(str,cropped2);
            count++;
        }
*/