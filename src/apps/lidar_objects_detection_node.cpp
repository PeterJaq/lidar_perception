#include <ros/ros.h>
#include "glog/logging.h"

using namespace lidar_perception;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "lidar_detection_node");
    ros::NodeHandle nh;

    std::string cloud_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");

    // subscribe to
    // a. raw fusioned pointcloud measurement
    // publish
    // b. detection objector
    std::shared_ptr<LidarDetectionFlow> lidar_detection_flow_ptr = std::make_shared<LidarDetectionFlow>(nh, cloud_topic);

    // pre-process lidar point cloud at 100Hz:
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        lidar_detection_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}