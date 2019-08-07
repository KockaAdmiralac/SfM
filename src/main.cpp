#include <iostream>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"

#include <kitti.hpp>
#include <ply.hpp>

// Minimum Hessian constant for SURF.
#define MIN_HESSIAN 400
// Lowe's ratio threshold.
#define RATIO_THRESH 0.7f
// Size of the hash array where matches are stored.
#define MAX_MATCHES 10000

/**
 * Matches descriptor vectors with a FLANN based matcher
 * and filters matches using the Lowe's ratio test.
 * @param matcher
 * @param descriptors1
 * @param descriptors2
 */
std::vector<cv::DMatch> extractMatches(cv::Ptr<cv::DescriptorMatcher> matcher, cv::Mat descriptors1, cv::Mat descriptors2)
{
    std::vector<cv::DMatch> matches;
    std::vector< std::vector<cv::DMatch> > knnMatchesList;
    matcher->knnMatch(descriptors1, descriptors2, knnMatchesList, 2);
    for (std::vector<cv::DMatch> knnMatches : knnMatchesList)
    {
        if (knnMatches[0].distance < RATIO_THRESH * knnMatches[1].distance)
        {
            matches.push_back(knnMatches[0]);
        }
    }
    return matches;
}

/**
 * Executed between every two frames.
 * @param image0 Left image of the first frame
 * @param image1 Right image of the first frame
 * @param image2 Left image of the second frame
 * @param image3 Right image of the second frame
 */
void frame(cv::Mat image0, cv::Mat image1, cv::Mat image2, cv::Mat image3, Sequence seq)
{
    // 1. FEATURE DETECTION
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(MIN_HESSIAN);
    std::vector<cv::KeyPoint> keypoints0, keypoints1, keypoints2, keypoints3;
    cv::Mat descriptors0, descriptors1, descriptors2, descriptors3;

    detector->detectAndCompute(image0, cv::noArray(), keypoints0, descriptors0);
    detector->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    detector->detectAndCompute(image3, cv::noArray(), keypoints3, descriptors3);

    // 2. FEATURE MATCHING
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector<cv::DMatch> matchesUp = extractMatches(matcher, descriptors0, descriptors1);
    std::vector<cv::DMatch> matchesDown = extractMatches(matcher, descriptors2, descriptors3);
    std::vector<cv::DMatch> matchesLeft = extractMatches(matcher, descriptors0, descriptors2);

    // 3. FEATURE MATCH PROCESSING
    cv::DMatch hashArrayUp[MAX_MATCHES];
    for (cv::DMatch match : matchesUp)
    {
        hashArrayUp[match.queryIdx].distance = match.distance;
        hashArrayUp[match.queryIdx].imgIdx   = match.imgIdx;
        hashArrayUp[match.queryIdx].queryIdx = match.queryIdx;
        hashArrayUp[match.queryIdx].trainIdx = match.trainIdx;
    }

    // 4. DETERMINING COMMON MATCHES
    std::vector <cv::Point2d> sharedKeypoints0, sharedKeypoints1, sharedKeypoints2;
    for (cv::DMatch match : matchesLeft)
    {
        // If the current match is present in both other matches:
        if (hashArrayUp[match.queryIdx].queryIdx != -1)
        {
            sharedKeypoints0.push_back(keypoints0[match.queryIdx].pt);
            sharedKeypoints1.push_back(keypoints1[hashArrayUp[match.queryIdx].trainIdx].pt);
            sharedKeypoints2.push_back(keypoints2[match.trainIdx].pt);
            //temp code:
            /*
                cv::circle(image0, keypoints0[match.queryIdx].pt, 31 , cv::Scalar(150,150,150), 7);
                cv::circle(image2, keypoints2[match.trainIdx].pt, 31 , cv::Scalar(150,150,150), 7); 
                cv::namedWindow("frame 0");
                cv::namedWindow("frame 1");
                cv::imshow("frame 0", image0);
                cv::imshow("frame 1", image2);
                cv::waitKey(0);
             */
            //temp code
        }
    }

    // 5. TRIANGUATION
    cv::Mat triangulatedPointsMat;
    std::vector<cv::Point3d> triangulatedPoints;
    VertexList plyVertices;
    triangulatePoints(seq.calib[0], seq.calib[1], sharedKeypoints0, sharedKeypoints1, triangulatedPointsMat);

    // Writing to a .ply file and determining triangulated points.
    for (int i = 0; i < triangulatedPointsMat.cols; ++i)
    {
        cv::Mat point;
        convertPointsFromHomogeneous(triangulatedPointsMat.col(i).t(), point);
        cv::Point3d tempPoint(point.at<double>(0), point.at<double>(1), point.at<double>(2));
        triangulatedPoints.push_back(tempPoint);
        //#temp code

            //let's see the coordinates of this point:
            std::cout << "\n temp point's coordinates: \n" << tempPoint << std::endl;
            
            cv::Mat Point3D(4, 1, CV_64F);
            Point3D.at<double>(0,0) = tempPoint.x; 
            Point3D.at<double>(1,0) = tempPoint.y;
            Point3D.at<double>(2,0) = tempPoint.z;
            Point3D.at<double>(3,0) = 1;

            cv::Mat Projection2D = seq.calib[0] * Point3D;

            std::cout << " and those project to \n" << Projection2D << std::endl;
            std::cout << " and should project to \n [" << sharedKeypoints0[i].x << ", " << sharedKeypoints0[i].y << "]" << std::endl;

            std::cout << " and when projecting values before convertPointsFromHomogeneous(): \n\n" << std::endl;
            std::cout << " temp point's coordinates: \n" << triangulatedPointsMat.col(i) << std::endl;
            
            cv::Mat tempMat =  seq.calib[0] * triangulatedPointsMat.col(i);
            cv::Mat tempMat2;

            convertPointsFromHomogeneous(tempMat.t(), tempMat2);

            std::cout << " after dividing " << tempMat2 << std::endl; 


            std::cout << " and should project to \n [" << sharedKeypoints0[i].x << ", " << sharedKeypoints0[i].y << "]" << std::endl;

            
        //#temp code_end
        Vertex v(point.at<double>(0), point.at<double>(1), point.at<double>(2));
        plyVertices.push_back(v);
    }
    writeply("triangulation.ply", plyVertices);

    // 6. Obtaining rotation and translation matrices.
    cv::Mat rotationVector(3, 1, cv::DataType<double>::type);
    cv::Mat translationVector(3, 1, cv::DataType<double>::type);
    cv::Mat distCoeffs(4, 1, cv::DataType<double>::type);
    cv::Mat cameraMatrix(3, 3, CV_64F);
    
    distCoeffs.at<double>(0,0) = 0;
    distCoeffs.at<double>(1,0) = 0;
    distCoeffs.at<double>(2,0) = 0;
    distCoeffs.at<double>(3,0) = 0;

    rotationVector.at<double>(0,0) = 0;
    rotationVector.at<double>(1,0) = 0;
    rotationVector.at<double>(2,0) = 0;
    
    translationVector.at<double>(0,0) = 0;
    translationVector.at<double>(1,0) = 0;
    translationVector.at<double>(2,0) = 0;
    
    std::cout << "\n\ndist coefs:\n" << distCoeffs << std::endl;

    for(int i=0 ; i<3 ; i++)
    {
        for(int j=0 ; j<3 ; j++)
        {
            cameraMatrix.at<double>(i, j) =  seq.calib[0].at<double>(i, j);
        }
    }
    //temp code:
        cv::Mat fakePoints(1,20,CV_64FC3);
        
            fakePoints.at<cv::Point3d>(0) = cv::Point3d(100, 100, 97);
            fakePoints.at<cv::Point3d>(1) =  cv::Point3d(200, 69, 200);
            fakePoints.at<cv::Point3d>(2) =  cv::Point3d(420, 300, 300);
            fakePoints.at<cv::Point3d>(3) =  cv::Point3d(150, 90, 100);
            fakePoints.at<cv::Point3d>(4) =  cv::Point3d(190, 70, 200);
            fakePoints.at<cv::Point3d>(5) =  cv::Point3d(300, 300, 150);
            fakePoints.at<cv::Point3d>(6) =  cv::Point3d(100, 300, 100);
            fakePoints.at<cv::Point3d>(7) =  cv::Point3d(100, 200, 100);
            fakePoints.at<cv::Point3d>(8) =  cv::Point3d(90, 60, 260);
            fakePoints.at<cv::Point3d>(9) =  cv::Point3d(220, 340, 112);
            fakePoints.at<cv::Point3d>(10) =  cv::Point3d(20, 210, 125);
            fakePoints.at<cv::Point3d>(11) =  cv::Point3d(300, 323, 612);
            fakePoints.at<cv::Point3d>(12) =  cv::Point3d(412, 120, 211);
            fakePoints.at<cv::Point3d>(13) =  cv::Point3d(162, 52, 771);
            fakePoints.at<cv::Point3d>(14) =  cv::Point3d(205, 90, 50);
            fakePoints.at<cv::Point3d>(15) =  cv::Point3d(326, 222, 111);
            fakePoints.at<cv::Point3d>(16) =  cv::Point3d(102, 333, 90);
            fakePoints.at<cv::Point3d>(17) =  cv::Point3d(201, 61, 120);
            fakePoints.at<cv::Point3d>(18) =  cv::Point3d(101, 156, 212);
            fakePoints.at<cv::Point3d>(19) =  cv::Point3d(200, 74, 561);


        cv::Mat fCameraMatrix(3,4, CV_64F), fCameraMatrixCUT(3,3, CV_64F);
        for(int i = 0; i<3;i++)
        {
            for(int j =0;j<4;j++)
            {
                if(i==j && i != 2)  fCameraMatrix.at<double>(i,j) = 700;
                else if( i == j) fCameraMatrix.at<double>(i,j) = 1;
                else fCameraMatrix.at<double>(i,j) = 0;
            }
        }

        std::vector<cv::Point2d> fakeProj0, fakeProj2;
        cv::Mat temp3D(4,1, CV_64F);
        cv::Mat temp2D(3,1, CV_64F); //3x1
        cv::Mat nonHomoPoint;
        cv::Mat transformationMatrix(4,4,CV_64F);

        transformationMatrix.at<double>(0, 0) = 1;
        transformationMatrix.at<double>(0, 1) = 0;
        transformationMatrix.at<double>(0, 2) = 0;
        transformationMatrix.at<double>(0, 3) = 0;
        
        transformationMatrix.at<double>(1, 0) = 0;
        transformationMatrix.at<double>(1, 1) = 1;
        transformationMatrix.at<double>(1, 2) = 0;
        transformationMatrix.at<double>(1, 3) = 0;
        
        transformationMatrix.at<double>(2, 0) = 0;
        transformationMatrix.at<double>(2, 1) = 0;
        transformationMatrix.at<double>(2, 2) = 1;
        transformationMatrix.at<double>(2, 3) = 10;
         
        transformationMatrix.at<double>(3, 0) = 0;
        transformationMatrix.at<double>(3, 1) = 0;
        transformationMatrix.at<double>(3, 2) = 0;
        transformationMatrix.at<double>(3, 3) = 1;

        for(int i =0; i< 20 ; i ++ )
        {
            temp3D.at<double>(0,0) = fakePoints.at<cv::Point3d>(i).x;
            temp3D.at<double>(1,0) = fakePoints.at<cv::Point3d>(i).y;
            temp3D.at<double>(2,0) = fakePoints.at<cv::Point3d>(i).z;
            temp3D.at<double>(3,0) = 1;
            
            temp2D = fCameraMatrix * temp3D;

            
            cv::convertPointsFromHomogeneous(temp2D.t(), nonHomoPoint);
            std::cout <<"fram0: "<< nonHomoPoint << std::endl;
            fakeProj0.push_back(cv::Point2d(nonHomoPoint.at<double>(0),nonHomoPoint.at<double>(1)));

            
            temp2D = fCameraMatrix * transformationMatrix * temp3D;

            
            cv::convertPointsFromHomogeneous(temp2D.t(), nonHomoPoint);
            std::cout << "frame1: "<< nonHomoPoint << std::endl << std::endl;
            fakeProj2.push_back(cv::Point2d(nonHomoPoint.at<double>(0),nonHomoPoint.at<double>(1)));
            
        }

    for(int i=0 ; i<3 ; i++)
    {
        for(int j=0 ; j<3 ; j++)
        {
            fCameraMatrixCUT.at<double>(i, j) =  fCameraMatrix.at<double>(i, j);
        }
    }

    std::vector<double> dumb;
    /*
    if (solvePnP(fakePoints,fakeProj2, fCameraMatrixCUT, dumb, rotationVector, translationVector))
    {
        printf("Successfully finished fake solvePNP()\n");
    }
    */
    
    cv::Mat butcheredTriangulatedPoints(1, triangulatedPoints.size(), CV_64FC3);
    for (int i = 0; i < triangulatedPoints.size(); ++i)
    {
        butcheredTriangulatedPoints.at<cv::Point3d>(i) = triangulatedPoints[i];
    }
    std::cout << butcheredTriangulatedPoints << triangulatedPoints << std::endl;

    if (solvePnPRansac(butcheredTriangulatedPoints, sharedKeypoints2, cameraMatrix, dumb, rotationVector, translationVector))
    {
        printf("Successfully finished solvePNP()\n");
    }

    cv::Mat rotationMatrix;
    Rodrigues(rotationVector, rotationMatrix);
    std::cout << "\n Rotation Matrix: \n" << rotationMatrix << std::endl;
    std::cout << "\n Translation Vector: \n" << translationVector << std::endl;
    std::cout << "\n ground truth Matrix: \n" << seq.poses.at(0) << std::endl;

    // 7.
}

int main()
{
    Sequence seq(0);
    for (int i = 0; i < 4541; ++i)
    {
        frame(
            seq.image(0, i),
            seq.image(1, i),
            seq.image(0, i + 1),
            seq.image(1, i + 1),
            seq
        );       
    }
    return 0;
}
