#include <iostream>
#include <opencv2/ximgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <kitti.hpp>
#include <ply.hpp>

// Minimum Hessian constant for SURF.
#define MIN_HESSIAN 400
// Lowe's ratio threshold.
#define RATIO_THRESH 0.7f

/**
 * Executed between every two frames.
 * @param imageLeft0 Left image of the first frame
 * @param imageRight0 Right image of the first frame
 * @param imageLeft1 Left image of the second frame
 * @param imageRight1 Right image of the second frame
 */
void frame(cv::Mat imageLeft0, cv::Mat imageRight0, cv::Mat imageLeft1, cv::Mat imageRight1, Sequence seq)
{
    // 1. FEATURE DETECTION.
    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(MIN_HESSIAN);
    std::vector<cv::KeyPoint> keypoints0, keypoints1, keypoints2, keypoints3;
    cv::Mat descriptors0, descriptors1, descriptors2, descriptors3;

    detector->detectAndCompute(imageLeft0, cv::noArray(), keypoints0, descriptors0);
    detector->detectAndCompute(imageRight0, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(imageLeft1, cv::noArray(), keypoints2, descriptors2);
    detector->detectAndCompute(imageRight1, cv::noArray(), keypoints3, descriptors3);

    // 2. FEATURE MATCHING
    // Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    std::vector< std::vector<cv::DMatch> > knnMatches0, knnMatches1, knnMatchesLeft;
    matcher->knnMatch(descriptors0, descriptors1, knnMatches0, 2);
    matcher->knnMatch(descriptors2, descriptors3, knnMatches1, 2);
    matcher->knnMatch(descriptors0, descriptors2, knnMatchesLeft, 2);

    // Filter matches using the Lowe's ratio test
    std::vector<cv::DMatch> matchesUp, matchesDown, matchesLeft;

    for (std::vector<cv::DMatch> knnMatches : knnMatches0)
    {
        if (knnMatches[0].distance < RATIO_THRESH * knnMatches[1].distance)
        {
            matchesUp.push_back(knnMatches[0]);
        }
    }
    for (std::vector<cv::DMatch> knnMatches : knnMatches1)
    {
        if (knnMatches[0].distance < RATIO_THRESH * knnMatches[1].distance)
        {
            matchesDown.push_back(knnMatches[0]);
        }
    }
    for (std::vector<cv::DMatch> knnMatches : knnMatchesLeft)
    {
        if (knnMatches[0].distance < RATIO_THRESH * knnMatches[1].distance)
        {
            matchesLeft.push_back(knnMatches[0]);
        }
    }

    // 3. FEATURE MATCH PROCESSING.
    std::vector<cv::Point2d> sharedKeypoints0, sharedKeypoints1, sharedKeypoints2;

    cv::DMatch hashArrayUp[10000];
    cv::DMatch hashArrayDown[10000];

    for (cv::DMatch match : matchesUp)
    {
        hashArrayUp[match.queryIdx].distance =  match.distance;
        hashArrayUp[match.queryIdx].imgIdx = match.imgIdx;
        hashArrayUp[match.queryIdx].queryIdx = match.queryIdx;
        hashArrayUp[match.queryIdx].trainIdx = match.trainIdx;
    }

    for (cv::DMatch match : matchesDown)
    {
        hashArrayDown[match.queryIdx].distance = match.distance;
        hashArrayDown[match.queryIdx].imgIdx = match.imgIdx;
        hashArrayDown[match.queryIdx].queryIdx = match.queryIdx;
        hashArrayDown[match.queryIdx].trainIdx = match.trainIdx;
    }

    std::vector <cv::Point2d> finalKP0,finalKP2,finalKP1,finalKP3;
    std::vector <cv::DMatch> finalMatches0, finalMatches1;
    cv::DMatch tempMatch0,tempMatch1;
    for(cv::DMatch match: matchesLeft)
    {
        if (hashArrayUp[match.queryIdx].queryIdx != -1 && hashArrayUp[match.queryIdx].trainIdx != -1) {
            tempMatch0.distance = hashArrayUp[match.queryIdx].distance;
            tempMatch0.imgIdx = hashArrayUp[match.queryIdx].imgIdx;
            tempMatch0.queryIdx = hashArrayUp[match.queryIdx].queryIdx;
            tempMatch0.trainIdx = hashArrayUp[match.queryIdx].trainIdx;
            finalMatches0.push_back(tempMatch0);
        }
        if (hashArrayDown[match.trainIdx].queryIdx != -1 && hashArrayDown[match.trainIdx].trainIdx != -1) {
            tempMatch1.distance = hashArrayDown[match.trainIdx].distance;
            tempMatch1.imgIdx = hashArrayDown[match.trainIdx].imgIdx;
            tempMatch1.queryIdx = hashArrayDown[match.trainIdx].queryIdx;
            tempMatch1.trainIdx = hashArrayDown[match.trainIdx].trainIdx;
            finalMatches1.push_back(tempMatch1);
        }

    }

    // 4. TRIANGUATION
    cv::Mat triangulatedPoints;
    triangulatePoints(seq.calib[0], seq.calib[1], finalKP0, finalKP1, triangulatedPoints); //
    VertexList vertices0, vertices1;
    std::vector<cv::Point3d> objectPoints;

    // Writing to a .ply file.
    for (int i = 0; i < triangulatedPoints.cols; ++i)
    {
        cv::Mat p3d;
        cv::Mat _p3h = triangulatedPoints.col(i);
        convertPointsFromHomogeneous(_p3h.t(), p3d);
        cv::Point3d tempPoint(p3d.at<double>(0), p3d.at<double>(1), p3d.at<double>(2));
        objectPoints.push_back(tempPoint);

        Vertex v(p3d.at<double>(0), p3d.at<double>(1), p3d.at<double>(2));
        vertices0.push_back(v);
    }
    writeply("triangulation.ply",vertices0);

    // 5. Obtaining rotation and translation matrices.
    cv::Mat rvec(3,1,cv::DataType<double>::type);
    cv::Mat tvec(3,1,cv::DataType<double>::type);
    cv::Mat distCoeffs(4,1,cv::DataType<double>::type);

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

    if(solvePnP(objectPoints,newKP2,cameraMatrix,distCoeffs,rvec,tvec))
    {
        printf("successfully finished solvePNP()");
    }

    cv::Mat rotationMat;
    Rodrigues(rvec,rotationMat);

    std::cout << "\n Rotation Matrix: \n" << rotationMat<<std::endl;
    std::cout << "\n Translation Vector: \n" << tvec << std::endl;

    std::cout << "\n ground truth Matrix: \n" << seq.poses.at(0) << std::endl;

    // 6.
}

int main()
{
    Sequence seq(0);
    for (int i = 0; i < 4541; ++i) {
        try {
            frame(
                seq.image(0, i),
                seq.image(1, i),
                seq.image(0, i + 1),
                seq.image(1, i + 1),
                seq
            );
        } catch (int n) {
            if(n == -1) {
                break;
            }
        }
    }
    return 0;
}