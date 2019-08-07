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
// Debug mode (commend it not using)



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
 * @param Sequence
 * @param frameNumber
 */
void frame(cv::Mat image0, cv::Mat image1, cv::Mat image2, cv::Mat image3, Sequence seq,int frameNumber, cv::Mat AbsoluteCameraPosition)
{
    // 1. FEATURE DETECTION using cv::SURF feature extractor
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(MIN_HESSIAN);
    std::vector<cv::KeyPoint> keypoints0, keypoints1, keypoints2, keypoints3;
    cv::Mat descriptors0, descriptors1, descriptors2, descriptors3;

    detector->detectAndCompute(image0, cv::noArray(), keypoints0, descriptors0);
    detector->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    detector->detectAndCompute(image3, cv::noArray(), keypoints3, descriptors3);

    // 2. FEATURE MATCHING using cv::FLANNBASED matcher
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
        Vertex v(point.at<double>(0), point.at<double>(1), point.at<double>(2));
        plyVertices.push_back(v);
    }
    writeply("triangulation.ply", plyVertices);

    // 6. Obtaining rotation and translation matrices.
    cv::Mat rotationVector(3, 1, cv::DataType<double>::type);
    cv::Mat translationVector(3, 1, cv::DataType<double>::type);
    cv::Mat cameraMatrix(3, 3, CV_64F);

    rotationVector.at<double>(0,0) = 0;
    rotationVector.at<double>(1,0) = 0;
    rotationVector.at<double>(2,0) = 0;

    translationVector.at<double>(0,0) = 0;
    translationVector.at<double>(1,0) = 0;
    translationVector.at<double>(2,0) = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            cameraMatrix.at<double>(i, j) =  seq.calib[0].at<double>(i, j);
        }
    }
    cv::Mat butcheredTriangulatedPoints(1, triangulatedPoints.size(), CV_64FC3);
    for (int i = 0; i < triangulatedPoints.size(); ++i)
    {
        butcheredTriangulatedPoints.at<cv::Point3d>(i) = triangulatedPoints[i];
    }

    std::vector<double> dumb;
    if (solvePnPRansac(butcheredTriangulatedPoints, sharedKeypoints2, cameraMatrix, dumb, rotationVector, translationVector))
    {
        printf("Successfully finished solvePNP()\n");
    }

    cv::Mat rotationMatrix;
    Rodrigues(rotationVector, rotationMatrix);

    #ifdef DEBUG_MODE
    std::cout << "\n Rotation Matrix: \n" << rotationMatrix << std::endl;
    std::cout << "\n Translation Vector: \n" << translationVector << std::endl;
    std::cout << "\n ground truth Matrix: \n" << seq.poses.at(frameNumber) << std::endl;
    #endif //DEBUG_MODE

    cv::Mat conc(4,4, CV_64F);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            conc.at<double>(i, j) = rotationMatrix.at<double>(i, j);
        }
    }
    conc.at<double>(0, 3) = translationVector.at<double>(0, 0);
    conc.at<double>(1, 3) = translationVector.at<double>(1, 0);
    conc.at<double>(2, 3) = translationVector.at<double>(2, 0);
    conc.at<double>(3, 0) = 0;
    conc.at<double>(3, 1) = 0;
    conc.at<double>(3, 2) = 0;
    conc.at<double>(3, 3) = 1;
    #ifdef DEBUG_MODE

    std::cout << "rotation matrix: \n" << rotationMatrix << std::endl;
    std::cout << "translation vector: \n " << translationVector << std::endl;
    std::cout << "CONC: \n" << conc << std::endl;
    std::cout <<"Camera position new =\n" << AbsoluteCameraPosition << std::endl << " conc.inv() = \n" << conc.inv() << std::endl;
    #endif


    AbsoluteCameraPosition = AbsoluteCameraPosition * conc.inv();


    std::cout << "Camera Postion new = \n" << AbsoluteCameraPosition << std::endl;



    // 7. Akomuliranje transformacija:


}

int main()
{
    Sequence seq(0);
    cv::Mat AbsoluteCameraPosition(4,4, CV_64F);
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (i == j)
            {
                AbsoluteCameraPosition.at<double>(i, j) = 1;
            }
            else
            {
                AbsoluteCameraPosition.at<double>(i, j) = 0;
            }
        }
    }

    for (int i = 0; i < 4541; ++i)
    {
        frame(
            seq.image(0, i),
            seq.image(1, i),
            seq.image(0, i + 1),
            seq.image(1, i + 1),
            seq,
            i,
            AbsoluteCameraPosition
        );
    }
    return 0;
}
