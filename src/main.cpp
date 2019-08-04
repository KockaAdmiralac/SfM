#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>

//#include "klt.hpp"
//#include "kanatani.hpp"
//#include "sift.hpp"
//#include "surf.hpp"
#include "kitti.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main() {
    Mat imageLeft;
    Mat imageRight;
    Sequence seq(0);
    int i = 0;
    //the main loop
    while(1)
    {
        // loop through images:
        i++;
        
        try
        {
            imageLeft = seq.image(0, i);
            imageRight = seq.image(1, i);
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

        Mat img_matches;

        //cout << "features1: " << newVec.at(i-1).queryIdx <<"with pos: " << keypoints1.at(newVec.at(i-1).queryIdx).pt << "\n";
        // cout << "features2: " << newVec.at(i-1).trainIdx <<"with pos: " << keypoints2.at(newVec.at(i-1).queryIdx).pt << "\n";
        
        //cout << "type = " << descriptors1.type() << "\n";
        
        drawMatches( imageLeft, keypoints1, imageRight, keypoints2, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        imshow("Good Matches", img_matches );

        waitKey(0);

        std::vector<Point2f> newKP1, newKP2;

        KeyPoint temp1;
        KeyPoint temp2;
        int N = good_matches.size();
        Mat pnts3D(1,N,CV_64FC4);

        for(cv::DMatch match : good_matches)
        {
            temp1 = keypoints1[match.queryIdx];
            temp2 = keypoints2[match.trainIdx];


            newKP1.push_back(temp1.pt);
            newKP2.push_back(temp2.pt);
        }

        
        triangulatePoints(seq.calib[0], seq.calib[1], newKP1, newKP2, pnts3D);
        cout << pnts3D << endl;
        waitKey(0);
        
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
