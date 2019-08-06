#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/calib3d.hpp>
//#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>

//#include "klt.hpp"
//#include "kanatani.hpp"
//#include "sift.hpp"
//#include "surf.hpp"
#include "kitti.hpp"
#include "ply.hpp"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

int main() {
    Mat imageLeft0; //at every point in loop, we need to load 2 frames 
    Mat imageRight0; // frame = (left_stereo_image,right_stereo_image)
    Mat imageLeft1;
    Mat imageRight1;
    Sequence seq(0);
    int i = 0;
    //the main loop
    while(1)
    {
        // loop through images:
        i++;
        
        try
        {   
            imageLeft0 = seq.image(0, i).clone();
            imageRight0 = seq.image(1, i).clone();
            imageLeft1 = seq.image(0, i+1).clone();
            imageRight1 = seq.image(1,i+1).clone();
        }
        catch(int n)
        {
            if(n == -1)
            {

                break;
            }
        }

    int minHessian = 400;
    Ptr<SURF> detector0 = SURF::create( minHessian );
    Ptr<SURF> detector1 = SURF::create( minHessian );
    Ptr<SURF> detector2 = SURF::create( minHessian );
    Ptr<SURF> detector3 = SURF::create( minHessian );
    std::vector<KeyPoint> keypoints0, keypoints1, keypoints2, keypoints3;

    Mat descriptors0, descriptors1,descriptors2,descriptors3;

    detector0->detectAndCompute( imageLeft0, noArray(), keypoints0, descriptors0 );
    detector1->detectAndCompute( imageRight0, noArray(), keypoints1, descriptors1 );
    detector2->detectAndCompute( imageLeft1, noArray(), keypoints2, descriptors2);
    detector3->detectAndCompute( imageRight1, noArray(), keypoints3, descriptors3);

    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used


    Ptr<DescriptorMatcher> matcher0 = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
    Ptr<DescriptorMatcher> matcher1 = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

    std::vector< std::vector<DMatch> > knn_matches0,knn_matches1;
    matcher0->knnMatch( descriptors0, descriptors1, knn_matches0, 2 );
    matcher1->knnMatch( descriptors2, descriptors3, knn_matches1, 2 );

    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    std::vector<DMatch> good_matches0, good_matches1, good_matches2;
    //cout <<"there are total of " << knn_matches0.size() << " matches in frame0";
    
    for (size_t i = 0; i < knn_matches0.size(); i++) //knn_matches.size()
    {
        if (knn_matches0[i][0].distance < ratio_thresh * knn_matches0[i][1].distance)
        {
            good_matches0.push_back(knn_matches0[i][0]);
        }
    }
    //cout << " and " << good_matches0.size() << "good matches \n";
    
    //cout <<"there are total of " << knn_matches1.size() << " matches in frame1";
    
    for (size_t i = 0; i < knn_matches1.size(); i++) //knn_matches.size()
    {
        if (knn_matches1[i][0].distance < ratio_thresh * knn_matches1[i][1].distance)
        {
            good_matches1.push_back(knn_matches1[i][0]);
        }
    }
    //cout << " and " << good_matches1.size() << " good matches \n";
    
    
    
    //-- Draw matches

        Mat img_matches0, img_matches1;

        ////cout << "features1: " << newVec.at(i-1).queryIdx <<"with pos: " << keypoints0.at(newVec.at(i-1).queryIdx).pt << "\n";
        // //cout << "features2: " << newVec.at(i-1).trainIdx <<"with pos: " << keypoints1.at(newVec.at(i-1).queryIdx).pt << "\n";
        
        ////cout << "type = " << descriptors0.type() << "\n";
        
        drawMatches( imageLeft0, keypoints0, imageRight0, keypoints1, good_matches0, img_matches0, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        drawMatches( imageLeft1, keypoints2, imageRight1, keypoints3, good_matches1, img_matches1, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
       // imshow("1",img_matches0); radi ovo 
       // imshow("2",img_matches1);
        
       // waitKey();
        //-- Show detected matches /
        std::vector<cv::Point2d> newKP0, newKP1, newKP2, newKP3;

        KeyPoint temp0;
        KeyPoint temp1;
        KeyPoint temp2;
        KeyPoint temp3;

        
        Mat pnts3D0;
        Mat pnts3D1;

        DMatch hashArray0[10000];
        DMatch hashArray1[10000];

        for(cv::DMatch match : good_matches0)
        {
            //DMatch nema preklopljen operator:
            hashArray0[match.queryIdx].distance =  match.distance;  //at every pair of key0 we put key1
            hashArray0[match.queryIdx].imgIdx = match.imgIdx;
            hashArray0[match.queryIdx].queryIdx = match.queryIdx;
            hashArray0[match.queryIdx].trainIdx = match.trainIdx;
            newKP0.push_back(keypoints0[match.queryIdx].pt);
            newKP1.push_back(keypoints1[match.trainIdx].pt);
        }

        for(cv::DMatch match : good_matches1)
        {
            hashArray1[match.queryIdx].distance = match.distance;
            hashArray1[match.queryIdx].imgIdx = match.imgIdx;
            hashArray1[match.queryIdx].queryIdx = match.queryIdx;
            hashArray1[match.queryIdx].trainIdx = match.trainIdx;
            newKP2.push_back(keypoints2[match.queryIdx].pt);
            newKP3.push_back(keypoints3[match.trainIdx].pt);
        }

        
        //between frames matching:
        
        Ptr<DescriptorMatcher> matcher2 = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);

        std::vector < std::vector <DMatch> > knn_matches_middle;

        matcher2->knnMatch(descriptors0,descriptors2,knn_matches_middle, 2 );

        for (size_t i = 0; i < knn_matches_middle.size(); i++) //knn_matches.size()
        {
            if (knn_matches_middle[i][0].distance < ratio_thresh * knn_matches_middle[i][1].distance)
            {
                good_matches2.push_back(knn_matches_middle[i][0]);
            }
        }

        Mat img_matches_middle;
        drawMatches( imageLeft0, keypoints0, imageLeft1, keypoints2, std::vector<cv::DMatch>(good_matches2.begin(), good_matches2.begin() + 20), img_matches_middle, Scalar::all(-1),
                 Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        //imshow("aa",img_matches_middle);
        //waitKey();
        
        
        std::vector <DMatch> finalMatches0, finalMatches1; //YAY BRUTE FORCE
        DMatch tempMatch0,tempMatch1;
        for(DMatch match: good_matches2)
        {
            if (hashArray0[match.queryIdx].queryIdx != -1 && hashArray0[match.queryIdx].trainIdx != -1) {
                tempMatch0.distance = hashArray0[match.queryIdx].distance;
                tempMatch0.imgIdx = hashArray0[match.queryIdx].imgIdx;
                tempMatch0.queryIdx = hashArray0[match.queryIdx].queryIdx;
                tempMatch0.trainIdx = hashArray0[match.queryIdx].trainIdx;
                finalMatches0.push_back(tempMatch0);
            }
            if (hashArray1[match.trainIdx].queryIdx != -1 && hashArray1[match.trainIdx].trainIdx != -1) {
                tempMatch1.distance = hashArray1[match.trainIdx].distance;
                tempMatch1.imgIdx = hashArray1[match.trainIdx].imgIdx;
                tempMatch1.queryIdx = hashArray1[match.trainIdx].queryIdx;
                tempMatch1.trainIdx = hashArray1[match.trainIdx].trainIdx;
                finalMatches1.push_back(tempMatch1);
            }
        }

        triangulatePoints(seq.calib[0], seq.calib[1], newKP0, newKP1, pnts3D0);
        triangulatePoints(seq.calib[0], seq.calib[1], newKP2, newKP3, pnts3D1);
        VertexList vertices0, vertices1;
        std::vector<Point3d> objectPoints; 
        for (int i = 0; i < pnts3D0.cols; ++i) {
            cv::Mat p3d;
            cv::Mat _p3h = pnts3D0.col(i);
            convertPointsFromHomogeneous(_p3h.t(), p3d);
            //cout << p3d << endl;

            Point3d tempPoint(p3d.at<double>(0), p3d.at<double>(1), p3d.at<double>(2));
            objectPoints.push_back(tempPoint);
            
            Vertex v(p3d.at<double>(0), p3d.at<double>(1), p3d.at<double>(2));
            vertices0.push_back(v);
        }
        writeply("ply0.ply",vertices0);
        for (int i = 0; i < pnts3D1.cols; ++i) {
            cv::Mat p3d;
            cv::Mat _p3h = pnts3D1.col(i);
            convertPointsFromHomogeneous(_p3h.t(), p3d);
            //cout << p3d << endl;
            Vertex v(p3d.at<double>(0), p3d.at<double>(1), p3d.at<double>(2));
            vertices1.push_back(v);
        }
        writeply("ply1.ply",vertices1);

        Mat image_matches_refined0;
        Mat image_matches_refined1;

        drawMatches(imageLeft0, keypoints0,imageRight0,keypoints1,std::vector<cv::DMatch>(finalMatches0.begin(), finalMatches0.begin() + 20), image_matches_refined0, Scalar::all(-1),
                Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

        drawMatches(imageLeft1,keypoints2,imageRight1,keypoints3,std::vector<cv::DMatch>(finalMatches1.begin(), finalMatches1.begin() + 20), image_matches_refined1, Scalar::all(-1),
                Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

 
        //objectPoint done
        //imagePoints done
        //cameraMatrix done
        //rvec, tvec ??

        cv::Mat rvec(3,1,cv::DataType<double>::type);
        cv::Mat tvec(3,1,cv::DataType<double>::type);
        cv::Mat distCoeffs(4,1,cv::DataType<double>::type);


        //cout << seq.calib[0] << endl;
        

        cv::Mat cameraMatrix(3,3,CV_64F);
        
        cameraMatrix.at<double>(0,0) = seq.calib[0].at<double>(0,0);
        cameraMatrix.at<double>(0,1) = seq.calib[0].at<double>(0,1);
        cameraMatrix.at<double>(0,2) = seq.calib[0].at<double>(0,2);

        cameraMatrix.at<double>(1,0) = seq.calib[0].at<double>(1,0);
        cameraMatrix.at<double>(1,1) = seq.calib[0].at<double>(1,1);
        cameraMatrix.at<double>(1,2) = seq.calib[0].at<double>(1,2);

        cameraMatrix.at<double>(2,0) = seq.calib[0].at<double>(2,0);
        cameraMatrix.at<double>(2,1) = seq.calib[0].at<double>(2,1);
        cameraMatrix.at<double>(2,2) = seq.calib[0].at<double>(2,2);


       //cout << cameraMatrix << endl;


        if(solvePnP(objectPoints,newKP1,cameraMatrix,distCoeffs,rvec,tvec)) {
            printf("successfully finished solvePNP()");
            
        }

        Mat rotationMat;
        Rodrigues(rvec,rotationMat);
        
        cout << "\n Rotation Matrix: \n" << rotationMat<<endl; 
        cout << "\n Translation Vector: \n" << tvec << endl;

        //cout << "\n ground truth Matrix: \n" << 
        /*
        if you wanna generate the optical flow of the video: 

        for (DMatch m : good_matches2) {
            Point2f point_old = keypoints0[m.queryIdx].pt;
            Point2f point_new = keypoints2[m.trainIdx].pt;
            circle(imageLeft0, point_old, 3, Scalar(0, 0, 255), 1);  
            circle(imageLeft1, point_old, 3, Scalar(0, 0, 255), 1);
            circle(imageLeft0, point_new, 3, Scalar(255, 0, 0), 1);  
            circle(imageLeft1, point_new, 3, Scalar(255, 0, 0), 1); 
            
            line(imageLeft0, point_old, point_new, Scalar(0, 255, 0), 2, 8, 0);
            line(imageLeft1, point_old, point_new, Scalar(0, 255, 0), 2, 8, 0);
        }

         */
        //namedWindow(std::string("opflo0").append(to_string(i)));
        //imshow(std::string("opflo0").append(to_string(i)),imageLeft0);
        //waitKey(0);
        //destroyWindow(std::string("opflo0").append(to_string(i)));
        //imwrite(std::string("TEMP/opflow/").append(to_string(i)).append(".png"),imageLeft0);
        //imageLeft0.release();
        





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
