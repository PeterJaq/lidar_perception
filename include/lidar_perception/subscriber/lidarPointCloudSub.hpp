#ifndef LIDAR_PERCEPTION_SUBSCRIBER_LIDARPOINTCLOUDSUB_HPP_
#define LIDAR_PERCEPTION_SUBSCRIBER_LIDARPOINTCLOUDSUB_HPP_

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "lidar_perception/sensor_data/lidarCloudData.hpp"

namespace lidar_perception {
class LidarPointCloudSubscriber {
  public:
    LidarPointCloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    LidarPointCloudSubscriber() = default;
    void ParseData(std::deque<CloudData>& deque_cloud_data);

  private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
    void dump_pclfile(CloudData& cloud_data);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<CloudData> new_cloud_data_;

    bool dump_pcd = false;
    std::string dump_pcd_path = "";

    std::mutex buff_mutex_;
};
}

#endif