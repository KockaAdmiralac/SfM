#include <iostream>
#include <fstream>

#include <opencv2/ximgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <kitti.hpp>
#include <ply.hpp>

/**
 * Constants
 */
// Minimum Hessian constant for SURF.
#define MIN_HESSIAN 400
// Lowe's ratio threshold.
#define RATIO_THRESH 1.2f
// Size of the hash array where matches are stored.
#define MAX_MATCHES 10000
// Uncomment this to activate more debug.
//#define DEBUG_MODE
// Uncomment this to generate optical flow images.
#define GENERATE_OPFLOW
// Threshold for filtering matches based on keypoints distance.
#define DISTANCE_THRESH 600
// Current KITTI sequence number.
#define SEQUENCE 1


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
 * 
 * @param frameNumber
 */
void frame(cv::Mat image0, cv::Mat image1, cv::Mat image2, cv::Mat image3, Sequence seq, int frameNumber, cv::Mat AbsoluteCameraPosition)
{
    // 1. FEATURE DETECTION using cv::SURF feature extractor
    cv::Ptr<cv::xfeatures2d::SIFT> detector = cv::xfeatures2d::SIFT::create(MIN_HESSIAN);
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
        // <temp>
        // Filtering matches by distance.
        int distance = std::pow((keypoints0[match.queryIdx].pt.x - keypoints2[match.trainIdx].pt.x), 2) +
                       std::pow((keypoints0[match.queryIdx].pt.y - keypoints2[match.trainIdx].pt.y), 2);
        if (distance > DISTANCE_THRESH * DISTANCE_THRESH)
        {
            continue;
        }
        // </temp>
        // If the current match is present in both other matches:
        if (hashArrayUp[match.queryIdx].queryIdx != -1)
        {
            sharedKeypoints0.push_back(keypoints0[match.queryIdx].pt);
            sharedKeypoints1.push_back(keypoints1[hashArrayUp[match.queryIdx].trainIdx].pt);
            sharedKeypoints2.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // 4.5. GENERATE OPTICAL FLOW IMAGES
    #ifdef GENERATE_OPFLOW
    cv::Mat opflowImage = image1.clone();
    char opflowFilePath[PATH_MAX];
    for (cv::DMatch m : matchesLeft)
    {
        cv::Point2f oldPoint = keypoints0[m.queryIdx].pt;
        cv::Point2f newPoint = keypoints2[m.trainIdx].pt; 
        // <temp>
        // Filtering matches by distance.
        float distance = std::pow((oldPoint.x - newPoint.x), 2) +
                         std::pow((oldPoint.y - newPoint.y), 2);
        if (distance > DISTANCE_THRESH * DISTANCE_THRESH)
        {
            continue;
        }
        // </temp>
        cv::circle(opflowImage, oldPoint, 3, cv::Scalar(0, 0, 255), 1);

        cv::circle(opflowImage, newPoint, 3, cv::Scalar(255, 0, 0), 1); 
        cv::line(opflowImage, oldPoint, newPoint, cv::Scalar(0, 255, 0), 2, 8, 0);
        #ifdef DEBUG_MODE
        printf("%f | %f %f | %f %f\n", distance, oldPoint.x, oldPoint.y, newPoint.x, newPoint.y);
        cv::imshow("opflow frame", opflowImage);
        cv::waitKey();
        #endif
    }
    sprintf(opflowFilePath, "TEMP/OPFLOW/opflow_%02d/%d.png", seq.number, frameNumber);
    cv::imwrite(opflowFilePath, opflowImage);
    #endif

    // 5. TRIANGULATION
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
    char triangulationFilePath[PATH_MAX];
    sprintf(triangulationFilePath, "TEMP/TRIANGULATION/%02d/%02d.ply", seq.number, frameNumber);
    writeply(triangulationFilePath, plyVertices);

    // 6. TRANSFORMATION MATRIX ESTIMATION
    cv::Mat rotationVector(3, 1, cv::DataType<double>::type);
    cv::Mat rotationMatrix;
    cv::Mat translationVector(3, 1, cv::DataType<double>::type);
    cv::Mat cameraMatrix(3, 3, CV_64F);
    cv::Mat butcheredTriangulatedPoints(1, triangulatedPoints.size(), CV_64FC3);
    std::vector<double> dummyDistortionCoefficients;
    cv::Mat concatenatedTransformationMatrix(4,4, CV_64F);

    // PROVERITI
    rotationVector.at<double>(0, 0) = 0;
    rotationVector.at<double>(1, 0) = 0;
    rotationVector.at<double>(2, 0) = 0;

    translationVector.at<double>(0, 0) = 0;
    translationVector.at<double>(1, 0) = 0;
    translationVector.at<double>(2, 0) = 0;
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            cameraMatrix.at<double>(i, j) =  seq.calib[0].at<double>(i, j);
        }
    }
    for (size_t i = 0; i < triangulatedPoints.size(); ++i)
    {
        butcheredTriangulatedPoints.at<cv::Point3d>(i) = triangulatedPoints[i];
    }
    // Obtaining rotation and translation vectors.
    if (!solvePnPRansac(butcheredTriangulatedPoints, sharedKeypoints2, cameraMatrix, dummyDistortionCoefficients, rotationVector, translationVector))
    {
        printf("solvePnPRansac() failed on frame %d of sequence %d.\n", frameNumber, seq.number);
    }

    // Turning rotation vector into rotation matrix.
    Rodrigues(rotationVector, rotationMatrix);

    // Concatenating rotation matrix and translation vector.
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            concatenatedTransformationMatrix.at<double>(i, j) = rotationMatrix.at<double>(i, j);
        }
    }
    concatenatedTransformationMatrix.at<double>(0, 3) = translationVector.at<double>(0, 0);
    concatenatedTransformationMatrix.at<double>(1, 3) = translationVector.at<double>(1, 0);
    concatenatedTransformationMatrix.at<double>(2, 3) = translationVector.at<double>(2, 0);
    concatenatedTransformationMatrix.at<double>(3, 0) = 0;
    concatenatedTransformationMatrix.at<double>(3, 1) = 0;
    concatenatedTransformationMatrix.at<double>(3, 2) = 0;
    concatenatedTransformationMatrix.at<double>(3, 3) = 1;

    // 7. TRANSFORMATION ACCUMULATION
    AbsoluteCameraPosition = AbsoluteCameraPosition * concatenatedTransformationMatrix.inv();

    // 8. WRITING RESULTS
    char matrixFilePath[PATH_MAX];
    sprintf(matrixFilePath, "TEMP/MATRICES/%02d.txt", seq.number);
    std::ofstream matrixFile(matrixFilePath, std::ios_base::app);
    matrixFile << AbsoluteCameraPosition.at<double>(0, 0)
               << " "
               << AbsoluteCameraPosition.at<double>(0, 1)
               << " "
               << AbsoluteCameraPosition.at<double>(0, 2)
               << " "
               << AbsoluteCameraPosition.at<double>(0, 3)
               << " "
               << AbsoluteCameraPosition.at<double>(1, 0)
               << " "
               << AbsoluteCameraPosition.at<double>(1, 1)
               << " "
               << AbsoluteCameraPosition.at<double>(1, 2)
               << " "
               << AbsoluteCameraPosition.at<double>(1, 3)
               << " "
               << AbsoluteCameraPosition.at<double>(2, 0)
               << " "
               << AbsoluteCameraPosition.at<double>(2, 1)
               << " "
               << AbsoluteCameraPosition.at<double>(2, 2)
               << " "
               << AbsoluteCameraPosition.at<double>(2, 3)
               << "\n";
    matrixFile.close();
    printf("Finished frame %d.\n", frameNumber);

    cv::Mat AbsoluteRotationMatrix(3,3,CV_64F);
    for(int i = 0 ; i < 3 ; i++)
    {
        for(int j = 0 ; j < 3 ; j++)
        {
            AbsoluteRotationMatrix.at<double>(i, j) = AbsoluteCameraPosition.at<double>(i, j);
        }
    }

}


int main()
{
    Sequence seq(SEQUENCE);
    cv::Mat AbsoluteCameraPosition(4,4, CV_64F);

    // Initializing position matrix.
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

    // Iterating through frames.
    for (int i = 0; i < seq.fileNumber-1 ; ++i)
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
