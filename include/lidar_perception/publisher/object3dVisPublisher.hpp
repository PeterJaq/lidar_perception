#ifndef LIDAR_PERCEPTION_PUBLISHER_OBJECT3DVISPUBLISHER_HPP_
#define LIDAR_PERCEPTION_PUBLISHER_OBJECT3DVISPUBLISHER_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "lidar_perception/sensor_data/lidarObjects3DData.hpp"

namespace lidar_perception {
    class Object3DVisPublisher{
        public:
            Object3DVisPublisher(ros::NodeHandle& nh,
                            std::string topic_name,
                            std::string frame_id,
                            size_t buff_size);
            Object3DVisPublisher() = default;

        private:
            void Publish(LidarObjects3DData& objects3d_ptr_output, double time);
            void Publish(LidarObjects3DData& objects3d_ptr_output);

            bool HasSubscribers();

        private:
            void PublishData(LidarObjects3DData& objects3d_ptr_output, ros::Time time);
        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;
            std::string frame_id_;
    };
}
#endif