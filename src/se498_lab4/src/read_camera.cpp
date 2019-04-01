#include "opencv2/opencv.hpp"

using namespace cv;

int main(int, char**)
{
    VideoCapture video_capture(0); // Open the default camera

    if(!video_capture.isOpened()){ // Check if we succeeded
    	return -1;
    }  

    Mat gray_image;
		Mat canny_image;

    namedWindow("gray image", 1);
		int kernelSize = 3;
		int lowThreshold = 35; //max = 100
		int highThreshold = 90; // low*2~3

    while(true)
    {
        Mat frame;
        video_capture >> frame; // Get a new frame from camera

        // OpenCV image processing 
        
        cvtColor(frame, gray_image, COLOR_BGR2GRAY);

        // To create an edge image you will use GaussianBlur() and Canny() function in OpenCV

				GaussianBlur( gray_image, gray_image, Size(kernelSize, kernelSize), 0, 0 );
				Canny(gray_image, canny_image, lowThreshold, highThreshold, kernelSize );


		    // Show processed image
        imshow("gray image", canny_image); 
        
        if(waitKey(30) >= 0) break;
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
