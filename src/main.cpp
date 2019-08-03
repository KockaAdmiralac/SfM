#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "klt.hpp"
#include "kanatani.hpp"
#include "sift.hpp"
#include "surf.hpp"

using namespace std;
using namespace cv;


int main() {    
    Mat image;
    string path = "../data_SfM/PiazzaBraNewCropped/"; //path to dataset from current location
    int i = 0;
    string full_path;

    //the main loop
    while(1) //loop while there are pictures example name: PiazzaBra - 000007.JPG
    {
        // loop through images:
        i++;

        full_path = path;
        full_path.append(to_string(i));
        full_path.append(".JPG");
        
        //cout <<"opening " << full_path << endl; 
        try
        {
            image = imread(full_path, IMREAD_COLOR);
            if(! image.data)
            {
                cout <<"error loading: " << full_path << "\n";
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


        SIFT(image);

        return 0;
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