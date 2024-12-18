#ifndef LANEDETECTOR_H
#define LANEDETECTOR_H

#include <iostream>
#include "FrameGrabber.h"  // Correct header inclusion
#include <opencv2/opencv.hpp>

class LaneDetector : public FrameGrabber // Public inheritance
{
private:
    cv::Mat FrameforLane;

    // Helper methods for each processing step
    cv::Mat convertToGray(const cv::Mat& input_image);
    cv::Mat applyGaussianBlur(const cv::Mat& input_image, const cv::Size& kernel_size, const cv::Size& sigma);
    cv::Mat detectEdges(const cv::Mat& input_image, double low_threshold, double high_threshold, int aperture_size);
    cv::Mat applyROI(const cv::Mat& input_image);
    std::vector<cv::Vec4i> detectLines(const cv::Mat& input_image, double rho, double theta, int threshold, double min_line_length, double max_line_gap);
    cv::Mat drawLines(const cv::Mat& input_image, const std::vector<cv::Vec4i>& lines);

public:
    LaneDetector();
    void processFrame() override; // Override the pure virtual function
    void run(); // Method to handle processing loop
};

#endif //LANEDETECTOR_H
