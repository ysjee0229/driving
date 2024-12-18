#include "driving/FrameGrabber.h"

using namespace std;
using namespace cv;

FrameGrabber::FrameGrabber()
    : nh(""), pnh("~"), idx(0)
{
    pnh.param<std::string>("demo_method", DEMOTYPE, "video"); // Default to "video"

    if (DEMOTYPE == "video") {
        subFrame = nh.subscribe("/usb_cam/image_raw", 1, &FrameGrabber::CamCB, this);
        ROS_INFO("FrameGrabber initialized for video stream.");
    } else if (DEMOTYPE == "image") {
        std::string datasetPath = "/home/a/project_ws/dataset/TUSimple/test_set/clips/0530/1492626047222176976_0/*";
        std::vector<cv::String> file_paths;
        cv::glob(datasetPath, file_paths);

        if (file_paths.empty()) {
            ROS_ERROR("No images found in the folder: %s", datasetPath.c_str());
        } else {
            ROS_INFO("Loading %zu images from dataset.", file_paths.size());
            for (const auto& path : file_paths) {
                cv::Mat image = cv::imread(path);
                if (image.empty()) {
                    ROS_ERROR("Could not open or find the image: %s", path.c_str());
                    continue;
                }
                images.push_back(image);
            }

            if (!images.empty()) {
                frame = images[idx].clone();
                ROS_INFO("First image loaded from dataset.");
            }
        }
    } else {
        ROS_ERROR("Wrong type : %s. Supported types are 'video' and 'image'.", DEMOTYPE.c_str());
    }
}

void FrameGrabber::CamCB(const sensor_msgs::Image::ConstPtr &msg) {
    try {
        frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void FrameGrabber::show(const std::string& name, const cv::Mat& frame, int waitkey) {
    if (!frame.empty()) {
        imshow(name, frame);
        if (waitKey(waitkey) == 'c') {
            imwrite("/home/a/project_ws/src/driving/frame.jpg", frame);
            ROS_INFO("Frame saved to /home/a/project_ws/src/driving/frame.jpg");
        }
    } else {
        ROS_WARN("Attempted to display an empty frame.");
    }
}

bool FrameGrabber::isFrameAvailable() {
    return !frame.empty();
}
