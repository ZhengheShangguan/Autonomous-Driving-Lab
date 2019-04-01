#include "lane_follow/img_processor.hpp"
namespace piCAM {

/*!
 * \brief piCAM::imgProcessor::imgProcessor
 * Constructor that initializes the subscriber for the camera topic
 *
 * @todo:
 * 1. Change the topic subscribed when working on PC.
 * 2. Declare a publisher for the control topic in header, and initialize here.
 *
 * \param nh
 */
imgProcessor::imgProcessor(ros::NodeHandle &nh)
{
  getPerspectiveTransform();
  CamSub = nh.subscribe("usb_cam/image_raw/compressed",10,&imgProcessor::compressedCallback,this);
  // pc uses usb_cam package to drive the camera
  // pc topic is : usb_cam/image_raw/compressed
  // raspberry pi camera uses raspicam_node package
  // raspicam_node topic is : raspicam_node/image/compressed
  // initialize the publisher here:
  // remember to declare it in the header file.

}

/*!
 * \brief imgProcessor::getPerspectiveTransform
 * This function initializes the perspective transformation and inverse perspective
 * transformation for the image. Perspective transformation will define the
 * transformation from front view to top-down view, and the inverse transform will
 * transform from the top-down view back to front view.
 *
 * The transformation is calculated by providing pixel locations that are assumed to
 * be of the same positon in the world frame but in different perspective. The function
 * will then generate a transformation that one can use to transform pixel position from
 * one perspective to another.
 *
 * Google CV::getPerspectiveTransform for more detail.
 *
 * Helper function void imgProcessor::perspectiveWarp(cv::Mat &in_out)
 * and void imgProcessor::invPerspectiveWarp(cv::Mat &in_out) are written to
 * transform the images, see helper function section for detail.
 *
 * @todo:
 * 1. find the pixel coordinate of the point that represent the same location
 * 2. Perspective transform should transform the front view to top-down view
 * 3. invperspectiveTransform should transform the top-down view back to front view
 *
 */
void imgProcessor::getPerspectiveTransform()
{
  cv::Point2f ptImg[4]; // points in image (float)
  cv::Point2f ptWorld[4]; // points in world (float)
  uint row= 410;
  uint col= 308;

  //Modify the point positions accordingly. Use variables row and col if needed.
  ptImg[0] = cv::Point2f(152,142);// in pixel
  ptWorld[0] = cv::Point2f(152,142);// in centermeter

  ptImg[1] = cv::Point2f(253,143);// in pixel
  ptWorld[1] = cv::Point2f(253,143);// in centermeter

  ptImg[2] = cv::Point2f(298,277);// in pixel
  ptWorld[2] = cv::Point2f(298,277);// in centermeter

  ptImg[3] = cv::Point2f(111,273);// in pixel
  ptWorld[3] = cv::Point2f(111,273);// in centermeter

  perspectiveTransform = cv::getPerspectiveTransform(ptImg,ptWorld);
  invperspectiveTransform = cv::getPerspectiveTransform(ptWorld,ptImg);
}


/*!
 * \brief imgProcessor::getHist
 * This function should generate a 1-D histogram that sums up all data in
 * each column. For example: a 2-D 2x4 mat data that look like
 * 1 2 3 4
 * 2 1 0 0
 * should generate a 1-D histogram of:
 * 3 3 3 4
 *
 * @todo:
 * 1. implement the histogram function.
 *
 * \param input
 * \return
 */
cv::Mat imgProcessor::getHist(const cv::Mat &input){
  cv::Mat sum(1,input.cols,CV_32F); // mat( rows cols type init) this is a 1-D data storage.
  sum = cv::Mat::zeros(sum.size(),CV_32F); // initialize them to zero.

  // Implement your code here:
  // you may find cv::accumulate to be useful, google for detail.


  return sum;
}

/*!
 * \brief imgProcessor::polyFit
 * This function does polynomial fitting for each detected line.
 * The variable "lines" is a vector of "vector of point"
 * "vector of point" or aka vPoint contain cv::Point's that have element x and y,
 * and can be accessd by .x .y format.
 *
 * for example, lines.at(0).at(0).x, the first at(0) means at line[0], the second .at(0)
 * means the first set of point(x,y) in line[0], the .x means the x element in the
 * respective point. (.y to access y element in that point)
 *
 * To check the size of each, use lines.size() and line.at(0).size() since they are vectors
 *
 * Recommend using iterators if you know what they are.
 *
 * x = A.colPivHouseholderQr().solve(b); this function solves x for the problem of A*x=b
 *
 * @todo:
 * 1. Construct A and b to find coefficients (x) for a second degree polynomial that best fit
 *   points (x1,y1) (x2,y2) ... (xn,yn) that are given. the solver for A*x=b is given.
 * 2. implement the code to solve coefficients from each set of points for each line.
 *
 * \param lines
 * \return
 */
imgProcessor::vPoint3f imgProcessor::polyFit(const std::vector<imgProcessor::vPoint>& lines){
  imgProcessor::vPoint3f lineCoeff;
  Eigen::Vector3f x = Eigen::Vector3f::Zero();

  // for each line do:
  for(auto i:lines){
    if( i.empty() ) continue; // if points on lane is empty, then move on,
    if( i.size()<3 ) continue; // if points on lane is not enough for 2nd order polyfit, move on.

    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(i.size(),3);          //column major < type, row, column>
    Eigen::VectorXf b = Eigen::VectorXf::Zero(i.size());
    for(size_t row = 0, rows = A.rows(); row<rows;++row){
      for(size_t col = 0, cols = A.cols(); col<cols;++col){
  //! @todo: ADD your code here to populate
        // TO access each point in line, do i[_index_of_the_point_]
        // each i is of type imgProcessor::vPoint which is a vector of points.
        A(row,col)=0; //place holder, modify accordingly
      }// end for each row, populate A

      b(row) = i[row].x; // populate B
    }
    x = A.colPivHouseholderQr().solve(b);
    lineCoeff.emplace_back(cv::Point3f(x(0),x(1),x(2)));
  }
  return lineCoeff;

}

/*!
 * \brief imgProcessor::findCurve
 * this function calls a recursive function to find points for each line.
 * It uses sliding window approach.
 * \param input
 * \return
 */
imgProcessor::vPoint3f imgProcessor::findCurve(cv::Mat &input){
  vPoint3f lineCoeff;
  cv::Rect region(0,input.rows-winRowSize,winColSize,winRowSize);
//  cv::Mat roi;// 82x44   (x,y, width height)
//  cv::Mat hist;//82x1
// cv::Rect roi = region & cv::Rect(0,0,input.width, input.height)
  //  roi = input(region);
  //  hist = getHist(roi);
  std::vector<vPoint> lines; // initialize empty line.

  findCurveRecur(input, region, lines, 0); // start looking for line at region with line 0.
  if( lines.empty()) return lineCoeff;  // if no line detected, return empty coefficient.
  lineCoeff = polyFit(lines);

  std::cout<< "*****************returned******************" <<std::endl;
  std::cout<< "number of lines: "<< lineCoeff.size() <<std::endl;
  for(auto i: lineCoeff){
    std::cout<<"line coefficient:"<< i <<std::endl;
  }
  cv::waitKey(10);
  return lineCoeff;
}

/*!
 * \brief imgProcessor::findCurveRecur
 * recursive function to find the points for each line.
 * This recursive function uses sliding region of interest method.
 * The behavior of this recursive function is as follows:
 * 0. The region of interest will start at the bottom left of the img (closest to the robot)
 * 1. The region of interest will traverse in img's horizontal direction until it finds a line segment.
 *    a. the function will insert a line, with its first point at the 1-D histogram's max position
 *    b. the function will call itself to check the region above the current ROI with it horizontal position recentered to the
 *       histogram's max position
 *    c. Recursive ends when it hits the top of the img, or shifted to the end of horizontal img size.
 * \param input
 * \param region
 * \param lines
 * \param line
 */

void imgProcessor::findCurveRecur(const cv::Mat &input, cv::Rect region, std::vector<vPoint> &lines, int line)
{
  cv::Rect newRegion = region & cv::Rect(0,0,input.cols, input.rows);
  if(newRegion.area()<= 100) // if region of interest is too small, then exit recursive.
    return;

  double min,max;
  cv::Point min_loc, max_loc;
  cv::Mat roi = input(newRegion); // get region of interest, no copy
  cv::Mat hist = getHist(roi);    // get histogram, pass by reference
  cv::minMaxLoc(hist,&min,&max,&min_loc,&max_loc);  //find the min/max location and its value.

  // ONLY IF the region of interest is at the bottom of the image, we increment the "line" counter
  // by one when we detect a line segment in the region of interest.
  if(newRegion.y == (input.rows-winRowSize)){
    // if ma of the histogram is greater than a arbitrary threshold, we consider the current
    // region of interest contains a line segment
    if(max> 250 ){
      if(line >= (lines.size()) ) // if line is more than size of line.
      {
        while(line >= lines.size()){
          lines.emplace_back();
        }
      }
/// @todo: uncomment code below and fill in the "?"
/// @todo: the "center" of the line segment (x,y) should be put into the lines.at(line) vector of points
/// @todo: when we do detect a line segment, check next region of interest above it with horizontal(x) region
///         shifted to the horizontal(x) center of the line segment.
/// @todo: if no line detecte, shift the region of interest (ROI) horizontally.

//      lines.at(line).push_back(cv::Point(?, ?)); //insert the point to the vector of lines.

//      findCurveRecur(input,
//                     region+cv::Point(?,?),
//                     lines,
//                     line+1);
    }
    else{
//      findCurveRecur(input,
//                     region+cv::Point(?,?),
//                     lines,
//                     line);
    }
   }

  if(max> 250 ) // arbitrary value, this means if we see a line, we shift up, regardless.
  {

    if(line >= (lines.size()) ) // if line is more than size of line.
    {
      while(line >= lines.size()){
        lines.emplace_back();
      }
    }
/// @todo: uncomment code below and fill in the "?"
/// @todo: the "center" of the line segment (x,y) should be put into the lines.at(line) vector of points
/// @todo: when we do detect a line segment, check next region of interest above it with horizontal(x) region
///         shifted to the horizontal(x) center of the line segment.

//    lines.at(line).push_back(cv::Point(?, ?)); //insert the point to the vector of lines.
//    findCurveRecur(input,
//                   region+max_loc+cv::Point(?,?), // recenter to max, and shift upward.
//                   lines,
//                   line);
  }
}

/*!
 * \brief imgProcessor::roadDectect
 * This is the main function
 * 1. using HLS to filter color
 * 2. using sobel/canny to find edge
 * 3. binary_and of edge and color detector's masks.
 * \param imgBGR
 * \return
 */
bool imgProcessor::roadDectect(cv::Mat &imgBGR)
{
  // do perspective warp for the img, Need to implement.
  perspectiveWarp(imgBGR);
  cv::imshow("warp", imgBGR);

  cv::Mat mask;
  cv::Mat imgHLS;
  cv::Mat combinedMask;
  vMat channelsHLS;
  uint8_t sx_min,sx_max;

  sx_min=15;
  sx_max=255;
  convertHLS(imgBGR,imgHLS,channelsHLS); //convert from BGR to HLS

  cv::Mat sobelx,scaled_sobelx;
  ///@todo: use sobel to find edges using channelsHLS[1], CV_64F, take derivative in x
  /// uncomment the line below and fill in ?
// cv::Sobel(channelsHLS[1],sobelx,CV_64F,?,?); // take the derivative in x direction
  cv::Sobel(channelsHLS[1],sobelx,CV_64F,1,1); // this is just a place holder, modify "?"
  ///@todo: normalized the sobelx data and store in scaled_sobelx
  /// you can use cv::abs() for absolute value
  /// uncomment the line below and fill in ?
//  cv::normalize(?, scaled_sobelx,?,?,cv::NORM_INF);
  cv::normalize(sobelx, scaled_sobelx,50,50,cv::NORM_INF); // place holder. NOT the solution

/// to view min max value of scaled_sobelx, uncomment the code section below
/*
  double min,max;
  cv::minMaxLoc(scaled_sobelx,&min,&max);
  std::cout<< max << "min" << min << std::endl;
*/

// display sobel, convert to 8bit.
  cv::convertScaleAbs(scaled_sobelx,scaled_sobelx); // convert from float to 8 bit unsigned.
  cv::imshow("sobel", scaled_sobelx);

// generate mask for the color selected.
  selectColor(imgHLS,mask,Color::WHITE);  // select color from the img
  cv::imshow("masked", mask);  // display the mask


// bitwise and of both masks
  cv::bitwise_and(scaled_sobelx,mask,combinedMask);

// you may want to use gaussian blur to reduce the noise
//  cv::GaussianBlur(combinedMask,combinedMask,cv::Size(7,7),1);
  cv::imshow("perspective", combinedMask);// display the mask
  vPoint3f lineCoeff = findCurve(combinedMask);
  drawCurve(lineCoeff,imgBGR);
  invPerspectiveWarp(imgBGR);
  cv::imshow("final",imgBGR);
  return true;
}

/*!
 * \brief imgProcessor::selectColor
 * Generate a mask to filter out unwanted edges give the color we want.
 * @todo: find the appropriate color lower and upper bound for each type.
 * @note: the color is defined in HLS, NOT BGR.
 *
 * \param input
 * \param mask
 * \param code
 */
void imgProcessor::selectColor(cv::Mat &input, cv::Mat &mask, Color code)
{
  cv::Vec3b lower;
  cv::Vec3b upper;

  switch(code){
  default:
    ROS_WARN("color selection error, default to white.");
  case Color::WHITE:
//    lower = cv::Vec3b(?,?,?);
//    upper = cv::Vec3b(?,?,?);
    break;
  case Color::YELLOW:
//    lower = cv::Vec3b(?,?,?);
//    upper = cv::Vec3b(?,?,?);
    break;
  case Color::BLACK:
//    lower = cv::Vec3b(?,?,?);
//    upper = cv::Vec3b(?,?,?);
    break;
  }

  cv::inRange(input,lower,upper,mask);  // output threshold based on the img.
  //cv::bitwise_and(input,input,input,mask);

}

void imgProcessor::grayScaling(cv::Mat& output)
{
}

///******************** HELPER functions below, read at your own time*******************************//

/*!
 * \brief imgProcessor::type2str
 * helper function to convert type to string.
 * \param type type defined CV_8U, etc...
 * \return
 */
std::string imgProcessor::type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

/*!
 * \brief piCAM::imgProcessor::compressedCallback
 * Compressed image callback function. It moves the img pointer to anther shared pointer
 * so it avoids copying the img itself everytime it comes in. The function getLastMat
 * will return this pointer when called. and if the pointer is empty that means theres no
 * image stored in the pointer.
 * \param img
 */
void imgProcessor::compressedCallback(const sensor_msgs::CompressedImage::ConstPtr &img)
{
  curCpImgPtr = std::move(img);
  //if(curCpImgPtr) std::cout<<"ptr is full of shit: "<< curCpImgPtr->data.size()<<std::endl;
}

/*!
 * \brief piCAM::imgProcessor::getLastMat
 * image.at<cv::Vec3b>(y,x); gives you the BGR vector of type cv::Vec3b
 * imdecode gives BGR data.
 * This functin will grab the last compressed image and decode it to BGR
 * image data format for us to process.
 * \return
 */
bool imgProcessor::getLastMat(cv::Mat &output)
{
  CIConstPtr lastPtr = std::move(curCpImgPtr);
  if(!lastPtr)
  {
   std::cout<<"last pointer is empty"<<std::endl;
   return false;
  }

  try
  {
    output = cv::imdecode(cv::Mat(lastPtr->data),1);
    // you can always uncomment line below to stream the output to window named "a"
    // but you have to create the namedwindow "a" first in the main file.
 //   cv::imshow("a", output);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert to image!");
  }
  return true;
}

/*!
 * \brief imgProcessor::perspectiveWarp
 * this function applies perspective warp on the input img
 * \param in_out
 */
void imgProcessor::perspectiveWarp(cv::Mat &in_out)
{
//  std::cout<< "cols: "<< in_out.cols << "; rows: " << in_out.rows <<std::endl;
  cv::warpPerspective(in_out,in_out,perspectiveTransform,in_out.size());
}

/*!
 * \brief imgProcessor::invPerspectiveWarp
 * this function applies inverse perspective warp on the input img
 * \param in_out
 */
void imgProcessor::invPerspectiveWarp(cv::Mat &in_out)
{
  cv::warpPerspective(in_out,in_out,invperspectiveTransform,in_out.size());
}

/*!
 * \brief imgProcessor::drawCurve
 * draw the curves on the img based on line coefficients and thickness.
 * \param lineCoeffs
 * \param input
 * \param thickness
 */
void imgProcessor::drawCurve(const vPoint3f lineCoeffs, cv::Mat &input, size_t thickness){
  int rows = input.rows;
  int cols = input.cols;
  for(auto i: lineCoeffs){
    for(int y = 0; y < rows; ++y){
      for(size_t t = 0; t < thickness; ++t){
        int x = i.x * y * y + i.y * y + i.z;
        if( x >=0 && x < cols)
        {
          input.at<cv::Vec3b>(y, std::min(std::max( int(x+t),0), cols) ) = cv::Vec3b(0,255,0);
          input.at<cv::Vec3b>(y, std::min(std::max( int(x-t),0), cols) ) = cv::Vec3b(0,255,0);
        }
      }
    }
  }
}

/*!
 * \brief imgProcessor::convertHLS
 * convert BGR to HLS
 * \param input
 * \param output
 * \param channels
 */
void imgProcessor::convertHLS(const cv::Mat &input, cv::Mat &output, vMat &channels)
{
  cv::cvtColor(input,output,cv::COLOR_BGR2HLS); //imgdecode returns BGR values.
  cv::split(output,channels);
}

} // END OF NAMESPACE piCAM
