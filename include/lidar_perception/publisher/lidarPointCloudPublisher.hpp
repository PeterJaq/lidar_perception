#ifndef LIDAR_PERCEPTION_PUBLISHER_LIDARPOINTCLOUDPUBLISHER_HPP_
#define LIDAR_PERCEPTION_PUBLISHER_LIDARPOINTCLOUDPUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "lidar_perception/sensor_data/lidarCloudData.hpp"

namespace lidar_perception {
    class LidarPointCloudPublisher{
        public:
            LidarPointCloudPublisher(ros::NodeHandle& nh,
                            std::string topic_name,
                            std::string frame_id,
                            size_t buff_size);
            LidarPointCloudPublisher() = default;

        public:
            void Publish(CloudData::CLOUD_PTR& cloud_ptr_input, double time);
            void Publish(CloudData::CLOUD_PTR& cloud_ptr_input);

            bool HasSubscribers();

        private:
            void PublishData(CloudData::CLOUD_PTR& cloud_ptr_input, ros::Time time);

        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;
            std::string frame_id_;
    };
}
#endif