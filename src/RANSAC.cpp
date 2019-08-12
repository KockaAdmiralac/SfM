#include "RANSAC.hpp"
#define DEBUG_MODE
ourRANSAC::ourRANSAC(int k, int N, int treshold, Sequence *seq)
{
    this->seq = seq;
    setRANSACParams(k, N, treshold);
}
void ourRANSAC::setImages(cv::Mat a, cv::Mat b)
{   
    cv::Mat temp_a,temp_b;
    cv::cvtColor(a,temp_a,cv::COLOR_GRAY2BGR);
    cv::cvtColor(b,temp_b,cv::COLOR_GRAY2BGR);

    this->image0 = temp_a.clone();
    this->image2 = temp_a.clone();

}
void ourRANSAC::setRANSACParams(int k, int N, int treshold)
{
    this->k = k;
    this->N = N;
    this->treshold = treshold;
}

void ourRANSAC::setRANSACArguments(cv::Mat trp, std::vector<cv::Point2d> skp,std::vector<cv::Point2d> skp0, cv::Mat camMatrix,
                            std::vector<double> DistortionCoefs, cv::Mat rotationVector, cv::Mat translationVector)
{   
    this->KeyPointsForEval = skp0;
    this->TriangulatedPointsAll = trp.clone();
    this->KeyPointsAll = skp;
    this->camMatrix = camMatrix.clone();
    this->DistortionCoefs = DistortionCoefs;
    this->rotationVector = rotationVector;
    this->translationVector = translationVector.clone();
}

void ourRANSAC::calculateExtrinsics()
{
    //1. Randomize subsets
    std::vector<int> randomArray;

    for(size_t i=0 ; i < KeyPointsAll.size() ; i++)
    {
        randomArray.push_back(i);
    }

    std::random_shuffle(randomArray.begin(), randomArray.end() ); //randomly shuffle

    for(size_t i = 0; i < this->N && i < randomArray.size(); i++)
    {
        int temp_index = randomArray[i];
        this->KeypointsSubset.push_back(KeyPointsAll[temp_index]);
        this->TriangulatedPointsSubset.push_back(TriangulatedPointsAll.at<cv::Point3d>(temp_index));
    }
    
    //temp code: test if shit's correct:

    //temp code end


    //2. SolvePnP with randomised substes
    cv::Mat rotationMatrix;   
    cv::Mat extrinsics(3,4, CV_64F);
    cv::Mat TriangulatedPointsSubsetNew;
    std::vector<cv::Point2d> KeypointsSubsetNew;
    for(int num=0;num<k;num++){
        cv::Mat visualisationMatrix0, visualisationMatrix2;
        visualisationMatrix0 = image0.clone();
        visualisationMatrix2 = image2.clone();

        //reseting both contianers:
        while(KeypointsSubsetNew.size() != 0)
            KeypointsSubsetNew.pop_back();
        
        TriangulatedPointsSubsetNew.release();
        TriangulatedPointsSubsetNew = cv::Mat();

        printf("sizes before solvePNP: %d , %d\n", TriangulatedPointsSubset.rows, KeypointsSubset.size());
        cv::solvePnP(TriangulatedPointsSubset,KeypointsSubset,this->camMatrix,this->DistortionCoefs,this->rotationVector,this->translationVector);

        //3. do The Rodrigues on Rotation vector

        cv::Rodrigues(rotationVector,rotationMatrix);

        //4. Concatenate Matrices
        

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                extrinsics.at<double>(i, j) = rotationMatrix.at<double>(i, j);
            }
            extrinsics.at<double>(i,3) = translationVector.at<double>(i, 0);
        }
        /*
        #ifdef DEBUG_MODE
            std::cout << "###################################\n";
            std::cout << "rotationVec: \n" << rotationVector << std::endl << "dimensions: " << rotationVector.rows << "x" << rotationVector.cols << std::endl;
            std::cout << "rotationMatrix: \n" << rotationMatrix << std::endl << "dimensions: " << rotationMatrix.rows << "x" << rotationMatrix.cols << std::endl;
            std::cout << "translatonVector: \n" << translationVector << std::endl;
            std::cout << "extrinsics: \n" << extrinsics << std::endl << "dimensions: " << extrinsics.rows << "x" << extrinsics.cols;
        #endif
        */
        //5. calculate projection for all points

        cv::Mat projForEval( 1,TriangulatedPointsAll.cols , CV_64FC3);
        int goodPoints = 0;
        for(int i=0; i<projForEval.cols; i++)
        {
            cv::Mat tempMat(4,1,CV_64F);
            cv::Point3d tempPoint = TriangulatedPointsAll.at<cv::Point3d>(i);
            cv::Mat tempResultMat;
            tempMat.at<double>(0,0) = tempPoint.x;
            tempMat.at<double>(1,0) = tempPoint.y;
            tempMat.at<double>(2,0) = tempPoint.z;
            tempMat.at<double>(3,0) = 1; 
            /*
            #ifdef DEBUG_MODE
                std::cout <<"tempMat\n" << tempMat<<std::endl;
                std::cout <<"dimensions\n" << tempMat.rows << "x" << tempMat.cols << std::endl;
                std::cout << "extrinsics dimensions: " << extrinsics.rows << "x" << extrinsics.cols << std::endl;
                std::cout << "tempMat dimensions:    " << tempMat.rows << "x" << tempMat.cols;
            #endif
            */
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
            
            tempResultMat = camMatrix * extrinsics * tempMat;
            cv::Point2d projectedPoint;
            projectedPoint.x = tempResultMat.at<double>(0,0) / tempResultMat.at<double>(2,0);
            projectedPoint.y = tempResultMat.at<double>(1,0) / tempResultMat.at<double>(2,0);
            
            double distance = (pow(projectedPoint.x - KeyPointsAll[i].x,2)) + (pow(projectedPoint.y - KeyPointsAll[i].y,2));
            #ifdef DEBUG_MODE               
                std::cout << "distance = " << sqrt(distance) << std::endl;
            #endif
            std::cout << "size of vector: " << KeypointsSubsetNew.size() << std::endl;
            if(distance < this->treshold * this->treshold - 2*num)
            {
                goodPoints++;
                TriangulatedPointsSubsetNew.push_back(TriangulatedPointsAll.at<cv::Point3d>(i));
                KeypointsSubsetNew.push_back(KeyPointsAll[i]);
                //#ifdef DEBUG_MODE
                    cv::circle(visualisationMatrix0, cv::Point(projectedPoint.x,projectedPoint.y), 1, cv::Scalar(0,0,255), 3);
                    cv::circle(visualisationMatrix2, KeyPointsAll[i], 1, cv::Scalar(255,0,0), 3);
                //#endif
            }
            //printf("pp = %f %f \n", projectedPoint.x , projectedPoint.y);
            //printf("gt = %f %f \n", KeyPointsForEval[i].x, KeyPointsForEval[i].y);
        }
        //#ifdef DEBUG_MODE
            cv::imshow("image2",visualisationMatrix0);
            cv::imshow("image0",visualisationMatrix2);
            cv::waitKey(0);
            std::cout << "good points: " << goodPoints << std::endl;
        //#endif

        if(KeypointsSubsetNew.size() == KeypointsSubset.size())
            break;

        TriangulatedPointsSubset = TriangulatedPointsSubsetNew.clone();
        KeypointsSubset = KeypointsSubsetNew;
    }
    
    this->finalRotationMatrix = rotationMatrix.clone();
    this->finalTranslationVector = translationVector.clone();
    std::cout << "finished RANSAC final extrinsics:\n ";
    std::cout << extrinsics << std::endl;
}

void ourRANSAC::returnValues(cv::Mat &rotationContainer, cv::Mat &translationContainer)
{
    rotationContainer = finalRotationMatrix.clone();
    translationContainer = finalTranslationVector.clone();
}


