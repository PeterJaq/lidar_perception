#ifndef LIDAR_PERCEPTION_PUBLISHER_OBJECT3DPERFPUBLISHER_HPP_
#define LIDAR_PERCEPTION_PUBLISHER_OBJECT3DPERFPUBLISHER_HPP_

#include <ros/ros.h>
// #include "lidar_perception/sensor_data/lidarObjects3DData.hpp"
#include "lidar_perception/sensor_data/lidarObjDet3DPerfData.hpp"
#include "lidar_perception_msgs/DetPerf.h"

namespace lidar_perception {
    class Object3DPerfPublisher{
        public:
            Object3DPerfPublisher(ros::NodeHandle& nh,
                            std::string topic_name,
                            size_t buff_size);
            Object3DPerfPublisher() = default;

        private:
            void Publish(LidarObjDet3DPerfData& objects3d_performance_ptr_output, double time);
            void Publish(LidarObjDet3DPerfData& objects3d_performance_ptr_output);

            bool HasSubscribers();

        private:
            void PublishData(LidarObjDet3DPerfData& objects3d_performance_ptr_output, ros::Time time);
        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;
            std::string frame_id_;
    };
}
#endif