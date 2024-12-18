#include "driving/LaneDetector.h"

using namespace std;
using namespace cv;

// Constructor
LaneDetector::LaneDetector() : FrameGrabber()
{
    ROS_INFO("LaneDetector initialized.");
}

// Convert to Grayscale
cv::Mat LaneDetector::convertToGray(const cv::Mat& input_image) {
    cv::Mat gray;
    cvtColor(input_image, gray, COLOR_BGR2GRAY);
    return gray;
}

// Apply Gaussian Blur
cv::Mat LaneDetector::applyGaussianBlur(const cv::Mat& input_image, const cv::Size& kernel_size, const cv::Size& sigma) {
    cv::Mat blurred;
    GaussianBlur(input_image, blurred, kernel_size, sigma.width);
    return blurred;
}

// Detect Edges using Canny
cv::Mat LaneDetector::detectEdges(const cv::Mat& input_image, double low_threshold, double high_threshold, int aperture_size) {
    cv::Mat edges;
    Canny(input_image, edges, low_threshold, high_threshold, aperture_size);
    return edges;
}

// Apply Region of Interest (ROI)
cv::Mat LaneDetector::applyROI(const cv::Mat& input_image) {
    // Define the polygon for ROI (a trapezoid)
    cv::Mat mask = cv::Mat::zeros(input_image.size(), input_image.type());
    std::vector<cv::Point> polygon;
    polygon.emplace_back(cv::Point(100, input_image.rows));
    polygon.emplace_back(cv::Point(input_image.cols - 100, input_image.rows));
    polygon.emplace_back(cv::Point(input_image.cols / 2 + 50, input_image.rows / 2));
    polygon.emplace_back(cv::Point(input_image.cols / 2 - 50, input_image.rows / 2));

    std::vector<std::vector<cv::Point>> pts = { polygon };
    cv::fillPoly(mask, pts, cv::Scalar(255));

    cv::Mat roi;
    cv::bitwise_and(input_image, mask, roi);
    return roi;
}

// Detect Lines using Probabilistic Hough Transform
std::vector<cv::Vec4i> LaneDetector::detectLines(const cv::Mat& input_image, double rho, double theta, int threshold, double min_line_length, double max_line_gap) {
    std::vector<cv::Vec4i> lines;
    HoughLinesP(input_image, lines, rho, theta, threshold, min_line_length, max_line_gap);
    return lines;
}

// Draw Lines on the Image
cv::Mat LaneDetector::drawLines(const cv::Mat& input_image, const std::vector<cv::Vec4i>& lines) {
    cv::Mat image_with_lines = input_image.clone();
    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        line(image_with_lines, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
    }
    return image_with_lines;
}

// Override processFrame to implement lane detection logic
void LaneDetector::processFrame() {
    if (isFrameAvailable()) {
        FrameforLane = frame.clone(); // Clone the current frame

        // Step 1: Grayscale Conversion
        cv::Mat gray = convertToGray(FrameforLane);

        // Step 2: Gaussian Blur
        cv::Mat blurred = applyGaussianBlur(gray, cv::Size(5, 5), cv::Size(1, 0));

        // Step 3: Canny Edge Detection
        cv::Mat edges = detectEdges(blurred, 128, 255, 3);

        // Step 4: Apply ROI
        cv::Mat roi = applyROI(edges);

        // Step 5: Detect Lines using Hough Transform
        std::vector<cv::Vec4i> lines = detectLines(roi, 1, CV_PI / 180, 50, 50, 150);

        // Step 6: Draw Lines
        cv::Mat result = drawLines(FrameforLane, lines);

        // Step 7: Show the Result
        show("Lane Detection", result, 1);
    }
    else {
        ROS_WARN("No frame available to process.");
    }

    // If DEMOTYPE is image, iterate through images
    if (DEMOTYPE == "image" && !images.empty()) {
        idx++;
        if (idx >= images.size()) {
            idx = 0; // Loop back to the first image or handle as needed
        }
        frame = images[idx].clone();
    }
}

// Run method to handle the processing loop
void LaneDetector::run() {
    if (DEMOTYPE == "video") {
        ROS_INFO("Starting ROS event loop for video processing.");
        while (ros::ok()) {
            ros::spinOnce(); // Process incoming ROS messages
            processFrame();  // Process the latest frame
            // Add a short delay to allow OpenCV to handle GUI events
            cv::waitKey(1);
        }
    }
    else if (DEMOTYPE == "image") {
        ROS_INFO("Starting image dataset processing.");
        while (ros::ok() && !images.empty()) {
            processFrame();
            // Add a delay to visualize the processing
            if (cv::waitKey(30) == 'q') { // Press 'q' to quit
                ROS_INFO("Image processing terminated by user.");
                break;
            }
        }
    }
    else {
        ROS_ERROR("Unsupported DEMOTYPE: %s", DEMOTYPE.c_str());
    }
}
