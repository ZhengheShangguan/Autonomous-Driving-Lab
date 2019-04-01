#include <stdio.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
 
int main( )
{
 
       Mat image;

       // LOAD image
       image = imread("/home/ros/catkin_zhenghe3/src/se498_lab4/images/tennisball.jpg", CV_LOAD_IMAGE_COLOR);  

       if(!image.data)  // Check for invalid input
       {
              cout <<  "Could not open or find the image" << std::endl ;
              return -1;
       }

       //DISPLAY image
       namedWindow( "window", CV_WINDOW_AUTOSIZE );   // Create a window for display.
       imshow("window", image);                       // Show our image inside it.

       //SAVE image to "/home/ros/catkin_netid/src/se498_lab4/images/copy_image.jpg"
				
       imwrite("/home/ros/catkin_zhenghe3/src/se498_lab4/images/copy_image.jpg", image);

       waitKey(0);                       // Wait for a keystroke in the window
       
       return 0;
}
