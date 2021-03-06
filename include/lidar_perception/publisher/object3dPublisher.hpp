/*
*/
#ifndef LIDAR_PERCEPTION_PUBLISHER_OBJECT3DPUBLISHER_HPP_
#define LIDAR_PERCEPTION_PUBLISHER_OBJECT3DPUBLISHER_HPP_

#include <ros/ros.h>
#include "lidar_perception_msgs/BoundingBoxes3D.h"
#include "lidar_perception/sensor_data/lidarObjects3DData.hpp"

namespace lidar_perception {
    class Object3DPublisher {
        public:
            Object3DPublisher(ros::NodeHandle& nh,
                            std::string topic_name,
                            std::string frame_id,
                            size_t buff_size);
            Object3DPublisher() = default;

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
} // namespace lidar_perception

#endif 