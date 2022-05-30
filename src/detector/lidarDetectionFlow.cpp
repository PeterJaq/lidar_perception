#include "lidar_perception/detector/lidarDetectionFlow.hpp"

#include "glog/logging.h"
#include "lidar_perception/global_defination/global_defination.h"

namespace lidar_perception{
    LidarDetectionFlow::LidarDetectionFlow(ros::NodeHandle& nh){
        nh.param<std::string>("cloud_topic", lidar_sub_topic, "/synced_cloud");

        nh.param<bool>("is_vis", is_vis, false);
        nh.param<std::string>("vis_obstacles_topic", vis_obstacles_topic, "/vis/obstacles");

        nh.param<bool>("is_performance", is_performance, false);
        nh.param<std::string>("performance_topic", performance_topic, "/performance/info/pointpillars");

        // subscriber
        lidar_point_cloud_sub_ptr_ = std::make_shared<LidarPointCloudSubscriber>(nh, lidar_sub_topic, 100000);

        // publisher 
        object_3d_pub_ptr_ = std::make_shared<Object3DPublisher>(nh, "/perception/lidar/obstacles", "odom", 100);

        if(is_vis){
            object_3d_vis_pub_ptr_ = std::make_shared<Object3DVisPublisher>(nh, "/vis/perception/obstacles", "odom", 100);
        }

        if(is_performance){
            object_detector_performance_pub_ptr_ = std::make_shared<Object3DPerfPublisher>(nh, "/log/performance/pointpillars", 100);
        }

        // model
        std::string model_file = "/home/pc/workspace/ws_lidar_perception/src/lidar_perception/onnxs/pointpillar.onnx";
        cudaStream_t stream = NULL;
        checkCudaErrors(cudaStreamCreate(&stream));
        pointpillar_detector_ptr_ = std::make_shared<PointPillar>(model_file, stream);
    }

    bool LidarDetectionFlow::Run(){
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

    // read data
    bool LidarDetectionFlow::ReadData(){
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

    bool LidarDetectionFlow::HasData(){
        if (cloud_data_buff_.size() == 0)
            return false;

        return true;
    }

    bool LidarDetectionFlow::ValidData(){
        current_cloud_data_ = cloud_data_buff_.front();
        cloud_data_buff_.pop_front();

        // TODO: feature to add obstacles speed, the obstacle need sync data with odom/imu
        return true;
    }

    bool LidarDetectionFlow::Infer(){

        std::vector<float> pointcloud_tmp;
        size_t points_size = current_cloud_data_.cloud_ptr->size();

        for (auto point: current_cloud_data_.cloud_ptr->points){
            pointcloud_tmp.emplace_back(point.x);
            pointcloud_tmp.emplace_back(point.y);                
            pointcloud_tmp.emplace_back(point.z);
            pointcloud_tmp.emplace_back(point.intensity);
        }
        float* points = &pointcloud_tmp[0];
        float *points_data = nullptr;
        unsigned int points_data_size = points_size * 4 * sizeof(float);
        checkCudaErrors(cudaMallocManaged((void **)&points_data, points_data_size));
        checkCudaErrors(cudaMemcpy(points_data, points, points_data_size, cudaMemcpyDefault));
        checkCudaErrors(cudaDeviceSynchronize());
        
        pointpillar_detector_ptr_->doinfer(points_data, points_size, nms_pred);
        
        Pred2Objects();
    
        nms_pred.clear();

        return true;
    }

    bool LidarDetectionFlow::PublishResults(){
        object_3d_pub_ptr_->Publish(objects_3d_, current_cloud_data_.time);
        if(is_vis){
            object_3d_vis_pub_ptr_->Publish(objects_3d_, current_cloud_data_.time);
        }

        return true;
    }

    void LidarDetectionFlow::Pred2Objects(){
        objects_3d_.objects3d_ptr->objects3d.clear();
        objects_3d_.objects3d_ptr->object_counter = 0;
        for(int i=0; i < nms_pred.size(); i++){
            Object3D obj3d;
            obj3d.x = nms_pred.at(i).x;
            obj3d.y = nms_pred.at(i).y;
            obj3d.z = nms_pred.at(i).z;
            obj3d.w = nms_pred.at(i).w; 
            obj3d.h = nms_pred.at(i).h; 
            obj3d.l = nms_pred.at(i).l; 
            obj3d.yaw = nms_pred.at(i).rt;
            obj3d.pitch = 0;
            obj3d.roll = 0;
            obj3d.probability = nms_pred.at(i).score;
            obj3d.cls = nms_pred.at(i).id;
            objects_3d_.objects3d_ptr->objects3d.emplace_back(obj3d);
            objects_3d_.objects3d_ptr->object_counter += 1;
        }
    }

} // namespace lidar_perception
