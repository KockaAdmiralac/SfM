#include <ransac.hpp>

//#define DEBUG_MODE
#define DISABLE_BREAK

ourRANSAC::ourRANSAC(int k, int N, int threshold, Sequence *seq)
{
    this->seq = seq;
    setRANSACParams(k, N, threshold);
}

#ifdef DEBUG_MODE
void ourRANSAC::setImages(cv::Mat a, cv::Mat b)
{
    cv::Mat temp_a,temp_b;
    cv::cvtColor(a,temp_a,cv::COLOR_GRAY2BGR);
    cv::cvtColor(b,temp_b,cv::COLOR_GRAY2BGR);

        this->image0 = temp_b.clone();
        this->image2 = temp_b.clone();
}
#endif
void ourRANSAC::setRANSACParams(int k, int N, int threshold)
{
    this->k = k;
    this->N = N;
    this->threshold = threshold;
}

void ourRANSAC::setRANSACArguments(cv::Mat trp, std::vector<cv::Point2d> skp, std::vector<cv::Point2d> skp0, cv::Mat camMatrix,
                            std::vector<double> DistortionCoefs, cv::Mat rotationVector, cv::Mat translationVector)
{
    this->KeyPointsForEval = skp0;
    this->TriangulatedPointsAll = trp.clone();
    this->KeyPointsAll = skp;
    this->camMatrix = camMatrix;
    this->DistortionCoefs = DistortionCoefs;
    this->rotationVector = rotationVector;
    this->translationVector = translationVector;
}

bool ourRANSAC::calculateExtrinsics()
{
    this->extrinsics.create(4,4,CV_64F);
    // 1. Randomize subsets
    std::vector<int> randomArray;

    for (size_t i = 0; i < KeyPointsAll.size(); i++)
    {
        randomArray.push_back(i);
    }

    std::random_shuffle(randomArray.begin(), randomArray.end());

    for (size_t i = 0; i < this->N && i < randomArray.size(); i++)
    {
        int temp_index = randomArray[i];
        this->KeypointsSubset.push_back(KeyPointsAll[temp_index]);
        this->TriangulatedPointsSubset.push_back(TriangulatedPointsAll.at<cv::Point3d>(temp_index));
    }

    // 2. SolvePnP with randomized subsets
    cv::Mat rotationMatrix;
    cv::Mat TriangulatedPointsSubsetNew;
    std::vector<cv::Point2d> KeypointsSubsetNew;
    for (int num = 0; num < k; num++)
    {
        #ifdef DEBUG_MODE
            cv::Mat visualisationMatrix0, visualisationMatrix2;
            visualisationMatrix0 = image0.clone();
            visualisationMatrix2 = image2.clone();
        #endif

        // Reseting both containers
        KeypointsSubsetNew.clear();
        TriangulatedPointsSubsetNew.release();
        TriangulatedPointsSubsetNew = cv::Mat();

        #ifdef DEBUG_MODE
            printf("sizes before solvePNP: %d , %d\n", TriangulatedPointsSubset.rows, KeypointsSubset.size());
            std::cout << "trpSubsize, kpsubsize = " << TriangulatedPointsSubset.rows << " " << KeypointsSubset.size() << std::endl;
        #endif

        while(1){ //stupidest way to continue after handling the exception
            try
            {
                cv::solvePnP(TriangulatedPointsSubset,KeypointsSubset, this->camMatrix, this->DistortionCoefs, this->rotationVector,this->translationVector,true);
            }
            catch(cv::Exception& e)
            {
                #ifdef DEBUG_MODE
                    std::cout <<"exception!!!!!!!!!!!!!!!!!!!" << std::endl;
                    std::cout << "##########################\n";
                    for(int i=0;i<3;i++)
                    {
                        printf("| ");
                        for(int j=0;j<4;j++)
                        {
                            printf("%.2f, ",this->extrinsics.at<double>(i,j));
                        }
                        printf("|\n");
                    }
                    std::cout << "#########################\n";
                    std::getchar();
                #endif
                try
                {
                    cv::solvePnPRansac(TriangulatedPointsSubset,KeypointsSubset, this->camMatrix, this->DistortionCoefs, this->rotationVector,this->translationVector,false);
                }
                catch (cv::Exception& e)
                {
                    return false;
                }
            }
            break;
        }
        // 3. Do Rodrigues on the rotation vector
        cv::Rodrigues(rotationVector,rotationMatrix);

        // 4. Concatenate matrices
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                this->extrinsics.at<double>(i, j) = rotationMatrix.at<double>(i, j);
            }
            this->extrinsics.at<double>(i,3) = translationVector.at<double>(i, 0)/2;
            this->extrinsics.at<double>(3,i) = 0;
        }
        this->extrinsics.at<double>(2,3) = translationVector.at<double>(2,0);
        this->extrinsics.at<double>(3,3) = 1;

        cv::Mat tempCamMat(4,4,CV_64F);
        for(int i = 0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                tempCamMat.at<double>(i,j) = camMatrix.at<double>(i,j);
            }
            tempCamMat.at<double>(3,i) = 0;
            tempCamMat.at<double>(i,3) = 0;
        }

        // 5. Calculate projection for all points
        cv::Mat projForEval( 1,TriangulatedPointsAll.cols , CV_64FC3);
        int goodPoints = 0;
        for (int i = 0; i < TriangulatedPointsAll.cols; i++)
        {  
            cv::Mat tempMat(4,1,CV_64F);
            cv::Point3d tempPoint = TriangulatedPointsAll.at<cv::Point3d>(i);
            cv::Mat tempResultMat;
            tempMat.at<double>(0,0) = tempPoint.x;
            tempMat.at<double>(1,0) = tempPoint.y;
            tempMat.at<double>(2,0) = tempPoint.z;
            tempMat.at<double>(3,0) = 1;
            cv::Mat projMatrix(4,4,CV_64F);
            for(int i =0;i<3;i++)
            {
                for(int j=0;j<4;j++)
                {
                    projMatrix.at<double>(i,j) = seq->calib[0].at<double>(i,j);
                }
            }
            projMatrix.at<double>(3,0) = 0;
            projMatrix.at<double>(3,1) = 0;
            projMatrix.at<double>(3,2) = 0;
            projMatrix.at<double>(3,3) = 1;

            tempResultMat = tempCamMat * this->extrinsics * tempMat;
        
            cv::Point2d projectedPoint;
            projectedPoint.x = tempResultMat.at<double>(0,0) / tempResultMat.at<double>(2,0);
            projectedPoint.y = tempResultMat.at<double>(1,0) / tempResultMat.at<double>(2,0);
            #ifdef DEBUG_MODE
                printf("%f, %f \n",projectedPoint.x, projectedPoint.y);
            #endif
            double distance = (pow(projectedPoint.x - KeyPointsAll[i].x,2)) + (pow(projectedPoint.y - KeyPointsAll[i].y,2));
            #ifdef DEBUG_MODE
                std::cout << "distance = " << sqrt(distance) << std::endl;
                std::cout << "size of vector: " << KeypointsSubsetNew.size() << std::endl;
            #endif
            if (distance < this->threshold * this->threshold - 2*num)
            {
                goodPoints++;
                TriangulatedPointsSubsetNew.push_back(TriangulatedPointsAll.at<cv::Point3d>(i));
                KeypointsSubsetNew.push_back(KeyPointsAll[i]);
                #ifdef DEBUG_MODE
                    cv::circle(visualisationMatrix0, cv::Point(projectedPoint.x,projectedPoint.y), 1, cv::Scalar(0,0,255), 3);
                    cv::circle(visualisationMatrix2, KeyPointsAll[i], 1, cv::Scalar(255,0,0), 3);
                #endif
            }
        }
        #ifdef DEBUG_MODE
            cv::imshow("image2",visualisationMatrix0);
            cv::imshow("image0",visualisationMatrix2);
            cv::waitKey(0);
            std::cout << "good points: " << goodPoints << std::endl;
        #endif
        #ifndef DISABLE_BREAK
            if (KeypointsSubsetNew.size() == KeypointsSubset.size())
            {
                break;
            }
        #endif
        TriangulatedPointsSubset = TriangulatedPointsSubsetNew;
        KeypointsSubset = KeypointsSubsetNew;
    }

    this->finalRotationMatrix = rotationMatrix.clone();
    this->finalTranslationVector = translationVector.clone();

    #ifdef DEBUG_MODE
        std::cout << "##########################################\n";
        for(int i=0;i<3;i++)
        {
            printf("| ");
            for(int j=0;j<4;j++)
            {
                printf("%.2f, ",this->extrinsics.at<double>(i,j));
            }
            printf("|\n");
        }
        std::cout << "##########################################\n";
    #endif
    return true;
}

void ourRANSAC::returnValues(cv::Mat &rotationContainer, cv::Mat &translationContainer)
{
    rotationContainer = finalRotationMatrix;
    translationContainer = finalTranslationVector;
}
    

