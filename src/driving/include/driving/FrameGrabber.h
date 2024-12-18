// FrameGrabber.h
#ifndef FRAMEGRABBER_H
#define FRAMEGRABBER_H

#include <iostream>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class FrameGrabber {
protected:
  std::string DEMOTYPE;        // Demo type: "video" or "image"

  ros::NodeHandle nh;
  ros::NodeHandle pnh;

  ros::Subscriber subFrame;
  cv::Mat frame;
  int idx;

  std::vector<cv::Mat> images; // Store loaded images if DEMOTYPE is "image"

public:
  FrameGrabber();
  virtual ~FrameGrabber() {}

  virtual void processFrame() = 0; // Pure virtual function to be implemented by derived classes

  // Utility functions
  void CamCB(const sensor_msgs::Image::ConstPtr& msg);
  void show(const std::string& name, const cv::Mat& frame, int waitkey);
  bool isFrameAvailable();

  // Getter methods
  const std::vector<cv::Mat>& getImages() const { return images; }
  const std::string& getDemoType() const { return DEMOTYPE; }
};

#endif //FRAMEGRABBER_H
