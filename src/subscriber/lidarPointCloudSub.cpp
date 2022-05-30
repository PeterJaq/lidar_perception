#include "lidar_perception/subscriber/lidarPointCloudSub.hpp"

#include "glog/logging.h"

namespace lidar_perception{
LidarPointCloudSubscriber::LidarPointCloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    nh_.param<bool>("dump_pcd", dump_pcd, false);
    nh_.param<std::string>("dump_pcd_path", dump_pcd_path, "PCDS");
    
    subscriber_ = nh_.subscribe(topic_name, buff_size, &LidarPointCloudSubscriber::msg_callback, this);
}

void LidarPointCloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    ROS_ERROR("Run Pointcloud msg cbk!");
    // convert ROS PointCloud2 to pcl::PointCloud<pcl::PointXYZ>:
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));
    // add new message to buffer:
    new_cloud_data_.push_back(cloud_data);
    if (dump_pcd){
        dump_pclfile(cloud_data);
    }
    buff_mutex_.unlock();
}

void LidarPointCloudSubscriber::dump_pclfile(CloudData& cloud_data){
    std::stringstream pcd_filename;
    pcd_filename << dump_pcd_path << "/" << std::to_string(cloud_data.time) << ".pcd";
    std::cout << pcd_filename.str() << std::endl;
    pcl::io::savePCDFileASCII<pcl::PointXYZI>(pcd_filename.str(), *cloud_data.cloud_ptr);
}

void LidarPointCloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();

    // pipe all available measurements to output buffer:
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    
    buff_mutex_.unlock();
}
} // namespace data_input