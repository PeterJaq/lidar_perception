#ifndef LIDAR_PERCEPTION_DETECTOR_LIDAR_DETECTION_FLOW
#define LIDAR_PERCEPTION_DETECTOR_LIDAR_DETECTION_FLOW

#include <ros/ros.h>
#include "lidar_perception/subscriber/lidarPointCloudSub.hpp"
#include "lidar_perception/tf_listener/tfListener.hpp"

#include "lidar_perception/publisher/object3dPublisher.hpp"

namespace lidar_perception{
    class LidarDetectionFlow{
        public:
            LidarDetectionFlow(ros::NodeHandle& nh, std::string objects3d_topic);

            bool Run();

        private:


        private:
        // subscriber 
            std::shared_ptr<LidarPointCloudSubscriber> lidar_point_cloud_sub_ptr_;
    }

} lidar_perception

#endif