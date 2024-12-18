// driving_node.cpp
#include "driving/LaneDetector.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "driving");
    ros::NodeHandle nh("~");

    LaneDetector ld;

    if (ld.getDemoType() == "video") {
        ROS_INFO("Starting ROS event loop for video processing.");
        while (ros::ok()) {
            ros::spinOnce(); // Process incoming ROS messages
            ld.processFrame(); // Process the latest frame
            // Add a short delay to allow OpenCV to handle GUI events
            cv::waitKey(1);
        }
    } else if (ld.getDemoType() == "image") {
        ROS_INFO("Starting image dataset processing.");
        const auto& images = ld.getImages();
        while (ros::ok() && !images.empty()) {
            ld.processFrame();
            // Add a delay to visualize the processing
            if (cv::waitKey(30) == 'q') { // Press 'q' to quit
                ROS_INFO("Image processing terminated by user.");
                break;
            }
        }
    } else {
        ROS_ERROR("Unsupported DEMOTYPE: %s", ld.getDemoType().c_str());
    }

    return 0;
}
