#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "klt.hpp"

using namespace std;
using namespace cv;


int main() {
    
    Mat image;
    string path = "../data_SfM/PiazzaBraNew/PiazzaBra - "; //path to dataset from current location
    int i = 0;
    string extra_path;
    string full_path;

    //the main loop
    while(1) //loop while there are pictures example name: PiazzaBra - 000007.JPG
    {
        // loop through images:
        i++; 

        if(i<10)        extra_path = "00000"; 
        else if(i<100)  extra_path = "0000";
        else if(i<1000) extra_path = "000";
        extra_path.append(to_string(i));
        
        full_path = path;
        full_path.append(extra_path);
        full_path.append(".JPG");
        
        cout <<"opening " << full_path << endl; 
        try
        {
            image = imread(full_path, IMREAD_GRAYSCALE);
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
        //namedWindow("Display window", WINDOW_AUTOSIZE);
        //imshow("Display window", image);


        //TODO:
            //SIFT
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
