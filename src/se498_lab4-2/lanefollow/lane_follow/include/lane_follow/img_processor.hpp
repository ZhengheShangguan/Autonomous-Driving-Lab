#pragma once
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include "opencv2/core.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <cmath>
namespace piCAM {

enum class Color
{
  WHITE,
  BLACK,
  YELLOW
};

class imgProcessor
{
public:
  typedef sensor_msgs::CompressedImage CI;
  typedef CI::ConstPtr  CIConstPtr;
  typedef std::vector<cv::Mat> vMat;
  typedef std::vector<cv::Point> vPoint;
  typedef std::vector<cv::Point3f> vPoint3f;

protected:
  //variables:
  ros::Subscriber CamSub;
  CIConstPtr curCpImgPtr;
  cv::Mat perspectiveTransform;
  cv::Mat invperspectiveTransform;
  int winColSize = 82;
  int winRowSize = 44;
  //functions:
  void compressedCallback(const sensor_msgs::CompressedImage::ConstPtr &img);
  void findCurveRecur(const cv::Mat& input, cv::Rect region, std::vector<vPoint>& lines, int line);
  vPoint3f polyFit(const std::vector<vPoint>& lines);
public:
  imgProcessor(ros::NodeHandle &nh);
  bool getLastMat(cv::Mat& output);
  bool roadDectect(cv::Mat& imgBGR);
  void convertHLS(const cv::Mat& input, cv::Mat& output, vMat& channels);
  void selectColor(cv::Mat& input, cv::Mat& mask, Color code = Color::WHITE);
  void grayScaling(cv::Mat& output);
  std::string type2str(int type);
  void perspectiveWarp(cv::Mat& in_out);
  void invPerspectiveWarp(cv::Mat& in_out);
  void getPerspectiveTransform();
  vPoint3f findCurve(cv::Mat& input);
  cv::Mat getHist(const cv::Mat& input);
  void drawCurve(const vPoint3f lineCoeffs, cv::Mat& input, size_t thickness = 3);

};

}

