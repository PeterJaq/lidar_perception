#include "lidar_perception/detector/lidarGroundFilterFlow.hpp"
#include <iostream>

#include "glog/logging.h"
#include "lidar_perception/global_defination/global_defination.h"

namespace lidar_perception{
    LidarGroundFilterFlow::LidarGroundFilterFlow(ros::NodeHandle& nh){
        nh.param<std::string>("cloud_topic", lidar_sub_topic, "/synced_cloud");
        nh.param<std::string>("ground_cloud_topic", ground_pointcloud_topic, "/synced_cloud");
        nh.param<std::string>("filted_cloud_topic", filtered_pointcloud_topic, "/synced_cloud");

        std::cout << lidar_sub_topic << " " << ground_pointcloud_topic << " " << filtered_pointcloud_topic  << std::endl;

        // subscriber
        lidar_point_cloud_sub_ptr_ = std::make_shared<LidarPointCloudSubscriber>(nh, lidar_sub_topic, 100000);

        // publisher
        ground_lidar_point_cloud_pub_ptr_ = std::make_shared<LidarPointCloudPublisher>(nh, ground_pointcloud_topic, "odom", 10);
        filtered_lidar_point_cloud_pub_ptr_ = std::make_shared<LidarPointCloudPublisher>(nh, filtered_pointcloud_topic, "odom", 10);
    
        // model
        ground_segmentation_ptr_ = std::make_shared<GroundPlaneFit>();

    }

    bool LidarGroundFilterFlow::Run(){
        if (!ReadData())
            return false;

        while(HasData()) {
            if (!ValidData())
                continue;

            Infer();
            PublishResults(); 
        }

        return true;
    }

    bool LidarGroundFilterFlow::ReadData(){
        // static std::deque<CloudData> 
        lidar_point_cloud_sub_ptr_ -> ParseData(cloud_data_buff_);

        if (cloud_data_buff_.size() == 0){

            return false;
        }

        double cloud_time = cloud_data_buff_.front().time;

        // TODO: feature to add obstacles speed, the obstacle need sync data with odom/imu
        static bool sensor_inited = false;
        if (!sensor_inited){
            sensor_inited = true;
        }

        return true;
    }

    bool LidarGroundFilterFlow::HasData(){
        if (cloud_data_buff_.size() == 0)
            return false;

        return true;
    }

    bool LidarGroundFilterFlow::ValidData(){
        current_cloud_data_ = cloud_data_buff_.front();
        cloud_data_buff_.pop_front();

        return true;
    }

    bool LidarGroundFilterFlow::Infer(){

        ground_segmentation_ptr_->mainLoop(current_cloud_data_.cloud_ptr, 
                                        filtered_cloud_data_.cloud_ptr,
                                        ground_cloud_data_.cloud_ptr);

        return true;
    }

    bool LidarGroundFilterFlow::PublishResults(){

        ground_lidar_point_cloud_pub_ptr_->Publish(ground_cloud_data_.cloud_ptr);
        filtered_lidar_point_cloud_pub_ptr_->Publish(filtered_cloud_data_.cloud_ptr);

        return true;
    }

}