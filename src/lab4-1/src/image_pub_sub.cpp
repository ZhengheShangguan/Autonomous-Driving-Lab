#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image Window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	public:

		ImageConverter():it_(nh_) {
			// Subscribe to input video feed and publish output video feed
			image_sub_ = it_.subscribe("/usb_camera_node/image_raw", 1, &ImageConverter::imageCallback, this);
			image_pub_ = it_.advertise("/image_converter/output_video", 1);
			cv::namedWindow(OPENCV_WINDOW);
		}

		~ImageConverter() {
			cv::destroyWindow(OPENCV_WINDOW);
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

			cv_bridge::CvImagePtr cv_ptr;

			try
			{
			  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e)
			{
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			}

			// Draw an example circle on the video stream
			if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
			  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
			}

			//640*480
			int x = 270;
			int y = 190;
			int width = 100;
			int height = 100;
			// our rectangle...
			cv::Rect r = cv::Rect(x, y, width, height);
<<<<<<< HEAD
			cv::Mat image;
			image = cv::imread("./image01.jpg");
			cv::imshow(OPENCV_WINDOW, image); 
			
=======
>>>>>>> 876d46e40d9df18f1c64b7c63c62c184c12db1ec
			// and its top left corner...
			cv::Point pt1(x, y);
			// and its bottom right corner.
			cv::Point pt2(x + width, y + height);
			// These two calls...
			//cv::rectangle(img, r, Scalar(0, 255, 0));
			cv::rectangle(cv_ptr->image, pt1, pt2, cv::Scalar(0, 255, 0),2);			

			// Update GUI Window
<<<<<<< HEAD
		
			//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
=======
			cv::imshow(OPENCV_WINDOW, cv_ptr->image);
>>>>>>> 876d46e40d9df18f1c64b7c63c62c184c12db1ec
			cv::waitKey(3);

			// Output modified video stream
			// Convert this message to a ROS sensor_msgs::Image message. 
			image_pub_.publish(cv_ptr->toImageMsg());
		}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
