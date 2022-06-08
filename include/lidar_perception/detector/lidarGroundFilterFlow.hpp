#ifndef LIDAR_PERCEPTION_DETECTOR_LIDARDETECTIONFLOW_HPP_
#define LIDAR_PERCEPTION_DETECTOR_LIDARDETECTIONFLOW_HPP_

#include <ros/ros.h>
#include "cuda_runtime.h"

#include "lidar_perception/subscriber/lidarPointCloudSub.hpp"
#include "lidar_perception/tf_listener/tfListener.hpp"

#include "lidar_perception/publisher/lidarPointCloudPublisher.hpp"
#include "lidar_perception/models/fastGroundSeg/ground_plane_fitting.h"

namespace lidar_perception{
    class LidarGroundFilterFlow{
        public:
            LidarGroundFilterFlow(ros::NodeHandle& nh);

            bool Run();

        private:
            bool ReadData();
            bool HasData();
            bool ValidData();
            bool Infer();
            bool PublishResults();
            void PredGround();

        private:
        // subscriber 
            std::shared_ptr<LidarPointCloudSubscriber> lidar_point_cloud_sub_ptr_;

        // publisher
            std::shared_ptr<LidarPointCloudPublisher> filtered_lidar_point_cloud_pub_ptr_;
            std::shared_ptr<LidarPointCloudPublisher> ground_lidar_point_cloud_pub_ptr_;
        // model
            std::shared_ptr<GroundPlaneFit> ground_segmentation_ptr_;

        // buff 
            std::deque<CloudData> cloud_data_buff_;

        // output data

        // data
            CloudData current_cloud_data_;
            CloudData ground_cloud_data_;
            CloudData filtered_cloud_data_;

        // param
            bool is_vis = false;
            bool is_performance = false;

            std::string lidar_sub_topic = "";
            std::string filtered_pointcloud_topic = "";
            std::string ground_pointcloud_topic = "";
            std::string performance_topic = "";
    };

} 

#endif