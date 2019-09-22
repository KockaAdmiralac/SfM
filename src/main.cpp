#include <ctime>
#include <iostream>
#include <fstream>

#include <opencv2/ximgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <kitti.hpp>
#include <ply.hpp>
#include <ransac.hpp>
/**
 * Constants
 */
// Minimum Hessian constant for SURF.
#ifndef MIN_HESSIAN
    #define MIN_HESSIAN 1000
#endif
// Lowe's ratio threshold.
#ifndef RATIO_THRESH
    #define RATIO_THRESH 0.8f
#endif
// Size of the hash array where matches are stored.
#define MAX_MATCHES 10000
// Uncomment this to activate more debug.
//#define DEBUG_MODE
// Uncomment this to generate optical flow images.
//#define GENERATE_OPFLOW
// Threshold for filtering matches based on keypoints distance.
#ifndef DISTANCE_THRESH
    #define DISTANCE_THRESH 600
#endif
// Current KITTI sequence number.
#ifndef SEQUENCE_NUMBER
    #define SEQUENCE_NUMBER 4
#endif
// RANSAC parameters
#ifndef RANSAC_1
    #define RANSAC_1 8
#endif
#ifndef RANSAC_2
    #define RANSAC_2 100
#endif
#ifndef RANSAC_3
    #define RANSAC_3 23
#endif
#ifndef DETECTOR
    #define DETECTOR xfeatures2d::SIFT
#endif
#ifndef MATCHING
    #define MATCHING FLANNBASED
#endif

/**
 * Initializes the absolute camera matrix,
 */
cv::Mat initializeCameraMatrix(Sequence &seq)
{
    cv::Mat result(4, 4, CV_64F);

    // Initializing position matrix.
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (i == j)
            {
                result.at<double>(i, j) = 1;
            }
            else
            {
                result.at<double>(i, j) = 0;
            }
        }
    }

    // Initializing the matrix file.
    char matrixFilePath[PATH_MAX];
    sprintf(matrixFilePath, "TEMP/MATRICES/%02d.txt", seq.number);
    std::ofstream matrixFile(matrixFilePath);
    for (int i = 0; i < 11; ++i)
    {
        matrixFile << seq.poses[0].at<double>(i) << " ";
    }
    matrixFile << seq.poses[0].at<double>(11) << "\n";
    matrixFile.close();

    return result;
}

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
 * @param seq
 * @param frameNumber
 * @param AbsoluteCameraPosition
 */
double frame(cv::Mat image0, cv::Mat image1, cv::Mat image2, cv::Mat image3, Sequence seq, int frameNumber, cv::Mat AbsoluteCameraPosition)
{
    // BEGIN PERFORMANCE MEASUREMENT
    clock_t startTime = clock();

    // 1. FEATURE DETECTION
    cv::Ptr<cv::DETECTOR> detector = cv::DETECTOR::create(MIN_HESSIAN);
    std::vector<cv::KeyPoint> keypoints0, keypoints1, keypoints2, keypoints3;
    cv::Mat descriptors0, descriptors1, descriptors2, descriptors3;

    detector->detectAndCompute(image0, cv::noArray(), keypoints0, descriptors0);
    detector->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
    detector->detectAndCompute(image3, cv::noArray(), keypoints3, descriptors3);

    // 2. FEATURE MATCHING using cv::FLANNBASED matcher
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::MATCHING);
    std::vector<cv::DMatch> matchesUp = extractMatches(matcher, descriptors0, descriptors1);
    std::vector<cv::DMatch> matchesDown = extractMatches(matcher, descriptors2, descriptors3);
    std::vector<cv::DMatch> matchesLeft = extractMatches(matcher, descriptors0, descriptors2);

    // 2.5. Showing matched features.
    #ifdef DEBUG_MODE
        cv::Mat outImage;
        cv::drawMatches(image0, keypoints0, image1, keypoints1, matchesUp, outImage);
        cv::imshow("matches", outImage);
        cv::waitKey(0);
    #endif


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
    triangulatePoints(seq.calib[0], seq.calib[1], sharedKeypoints0, sharedKeypoints1, triangulatedPointsMat);

    // 5.5. Determining triangulated points.
    for (int i = 0; i < triangulatedPointsMat.cols; ++i)
    {
        cv::Mat point;
        convertPointsFromHomogeneous(triangulatedPointsMat.col(i).t(), point);
        cv::Point3d tempPoint(point.at<double>(0), point.at<double>(1), point.at<double>(2));
        triangulatedPoints.push_back(tempPoint);
    }

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

    // 6.1. Obtaining rotation and translation vectors.
    #ifdef USE_THEIR_SOLVEPNP
        if (!solvePnPRansac(butcheredTriangulatedPoints, sharedKeypoints2, cameraMatrix, dummyDistortionCoefficients, rotationVector, translationVector))
        {
            printf("solvePnPRansac() failed on frame %d of sequence %d.\n", frameNumber, seq.number);
        }

        // Turning rotation vector into rotation matrix.
        Rodrigues(rotationVector, rotationMatrix);
    #else
        ourRANSAC ransac(RANSAC_1, RANSAC_2, RANSAC_3, &seq);
        #ifdef DEBUG_MODE
            ransac.setImages(image0,image2);
        #endif
        ransac.setRANSACArguments(
            butcheredTriangulatedPoints,
            sharedKeypoints2,
            sharedKeypoints0,
            cameraMatrix,
            dummyDistortionCoefficients,
            rotationVector,
            translationVector
        );
        ransac.calculateExtrinsics();
        ransac.returnValues(rotationMatrix,translationVector);

        #ifdef DEBUG_MODE
            std::cout << "#############################" << std::endl;
            std::cout << "rotationVector\n" << rotationMatrix << std::endl;
            std::cout << "translationVector\n" << translationVector << std::endl;
        #endif
    #endif

    // 6.2. Concatenating rotation matrix and translation vector.
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

    // END PERFORMANCE MEASUREMENT
    clock_t endTime = clock();

    // 8. WRITING RESULTS
    
    char matrixFilePath[PATH_MAX];
    char triangulationFilePath[PATH_MAX];
    double elapsedTime = double(endTime - startTime) / CLOCKS_PER_SEC;
    sprintf(matrixFilePath, "TEMP/MATRICES/%02d.txt", seq.number);
    sprintf(triangulationFilePath, "TEMP/TRIANGULATION/%02d/%02d.ply", seq.number, frameNumber);
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
    writeply(triangulationFilePath, triangulatedPoints);
    printf("Finished frame %d in %f seconds.\n", frameNumber, elapsedTime);

    // Return performance measurement.
    return elapsedTime;
}

void printPerformance(Sequence &seq, std::vector<double> &performance)
{
    double total = 0, max = 0, min = INT64_MAX, avg;
    char performanceFilePath[PATH_MAX];
    char metricsFilePath[PATH_MAX];
    sprintf(performanceFilePath, "TEMP/PERFORMANCE/%02d.txt", seq.number);
    sprintf(metricsFilePath, "TEMP/METRICS/%02d.txt", seq.number);
    std::ofstream performanceFile(performanceFilePath);
    std::ofstream metricsFile(metricsFilePath);

    for (double measurement : performance)
    {
        performanceFile << measurement << "\n";
        total += measurement;
        if (measurement > max)
        {
            max = measurement;
        }
        if (measurement < min)
        {
            min = measurement;
        }
    }
    avg = total / performance.size();
    performanceFile.close();

    printf("#----------------------------------------------------------#\n");
    printf("# Total: %.5f s                                      #\n", total);
    printf("# Max:   %.5f s                                         #\n", max);
    printf("# Min:   %.5f s                                         #\n", min);
    printf("# Avg:   %.5f s                                         #\n", avg);
    printf("#----------------------------------------------------------#\n");
    metricsFile << "Total: " << total << " s\nMax: " << max << " s\nMin: " << min << " s\nAvg: " << avg << " s\n";
    metricsFile.close();
}

int main()
{
    Sequence seq(SEQUENCE_NUMBER);
    cv::Mat AbsoluteCameraPosition = initializeCameraMatrix(seq);

    // Iterating through frames.
    std::vector<double> performance;
    for (int i = 0; i < seq.fileNumber-1 ; ++i)
    {
        performance.push_back(frame(
            seq.image(0, i),
            seq.image(1, i),
            seq.image(0, i + 1),
            seq.image(1, i + 1),
            seq,
            i,
            AbsoluteCameraPosition
        ));
    }
    printPerformance(seq, performance);

    return 0;
}
