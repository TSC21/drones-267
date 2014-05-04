#include "Corners.h"
#include <iostream>

using namespace std;

#define ESCAPE_KEY 27

int main(int argc, char **argv)
{
    char const* inputWindowHandle = "Input";
    char const* thresholdedWindowHandle = "Thresholded Image";
    char const* contourWindowHandle = "Contours";
    namedWindow(inputWindowHandle, CV_WINDOW_AUTOSIZE);
    // namedWindow(thresholdedWindowHandle, CV_WINDOW_AUTOSIZE);
    namedWindow(contourWindowHandle, CV_WINDOW_AUTOSIZE);

    Mat frame;

    int imageIndex = 1;
    while(true) 
	{
        frame = imread(argv[imageIndex]);
        if (frame.empty())
        {
            cout << "Couldn't load input image" << endl;
			break;
        }
        imageIndex++;

		if (imageIndex == argc)
			imageIndex = 1;

        Mat_<double> corners = detectCorners(
            frame,
            NULL,
            inputWindowHandle,
            NULL, // thresholdedWindowHandle,
            contourWindowHandle);
        cout << corners << endl << endl;
        
        if (corners.empty())
        {
        	Mat contourImg = Mat::zeros(frame.size(), CV_8UC3);
			imshow(contourWindowHandle, contourImg);
		}

        if ((char)waitKey(0) == ESCAPE_KEY)
            break;
    }
    destroyAllWindows();
    return 0;
}
