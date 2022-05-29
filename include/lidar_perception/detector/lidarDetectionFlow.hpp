#ifndef LIDAR_PERCEPTION_DETECTOR_LIDARDETECTIONFLOW_HPP_
#define LIDAR_PERCEPTION_DETECTOR_LIDARDETECTIONFLOW_HPP_

#include <ros/ros.h>
#include "cuda_runtime.h"

#include "lidar_perception/subscriber/lidarPointCloudSub.hpp"
#include "lidar_perception/tf_listener/tfListener.hpp"

#include "lidar_perception/publisher/object3dPublisher.hpp"
#include "lidar_perception/publisher/object3dVisPublisher.hpp"
#include "lidar_perception/publisher/object3dPerfPublisher.hpp"

#include "lidar_perception/models/pointpillarsRT/pointpillar.h"

#include "lidar_perception/sensor_data/lidarCloudData.hpp"
#include "lidar_perception/sensor_data/lidarObjects3DData.hpp"

namespace lidar_perception{
    class LidarDetectionFlow{
        public:
            LidarDetectionFlow(ros::NodeHandle& nh);

            bool Run();

        private:
            bool ReadData();
            bool HasData();
            bool ValidData();
            bool Infer();
            bool PublishResults();
            void Pred2Objects();

        private:
        // subscriber 
            std::shared_ptr<LidarPointCloudSubscriber> lidar_point_cloud_sub_ptr_;

        // publisher
            std::shared_ptr<Object3DPublisher> object_3d_pub_ptr_;
            std::shared_ptr<Object3DVisPublisher> object_3d_vis_pub_ptr_;
            std::shared_ptr<Object3DPerfPublisher> object_detector_performance_pub_ptr_;

        // model
            std::shared_ptr<PointPillar> pointpillar_detector_ptr_;

        // buff 
            std::deque<CloudData> cloud_data_buff_;

        // output data
            LidarObjects3DData objects_3d_;
            std::vector<Bndbox> nms_pred;

        // data
            CloudData current_cloud_data_;

        // param
            bool is_vis = false;
            bool is_performance = false;

            std::string lidar_sub_topic = "";
            std::string vis_obstacles_topic = "";
            std::string performance_topic = "";
    };

} 

#endif