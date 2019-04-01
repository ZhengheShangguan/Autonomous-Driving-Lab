#include "lane_follow/img_processor.hpp"
#include <iostream>
int main(int ac, char** av)
{
  ros::init(ac, av, "LaneFollowNode");
  ros::NodeHandle nh;
 // cv::namedWindow("a",cv::WINDOW_AUTOSIZE);// Create a window for display.
  cv::namedWindow("final",cv::WINDOW_AUTOSIZE);// Create a window for display.
  cv::namedWindow("perspective",cv::WINDOW_AUTOSIZE);
  cv::namedWindow("sobel",cv::WINDOW_AUTOSIZE);
  cv::namedWindow("masked",cv::WINDOW_AUTOSIZE);
  cv::namedWindow("warp",cv::WINDOW_AUTOSIZE);

  piCAM::imgProcessor a(nh);
  cv::Mat output;
  ros::Rate r(9);

  while(ros::ok()){
    ros::spinOnce();
    r.sleep();

   if(!a.getLastMat(output)) continue;
//   cv::imshow("a", output);
   if(output.empty()) continue;
   if(!output.empty()) cv::resize(output, output, cv::Size(410,308), 0, 0, CV_INTER_LINEAR);

   if(! a.roadDectect(output)) continue; // if no img, do nothing.
   //cv::imshow("b", output);

   cv::waitKey(5); // for some weird reason this is needed for namedwindow to show.

  }
  cv::destroyAllWindows();
  return 0;
}
