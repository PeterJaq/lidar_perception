#include "lidar_perception/publisher/object3dPerfPublisher.hpp"
#include "glog/logging.h"

namespace lidar_perception {
    Object3DPerfPublisher::Object3DPerfPublisher(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         size_t buff_size)
        :nh_(nh) {
        publisher_ = nh_.advertise<lidar_perception_msgs::DetPerf>(topic_name, buff_size);
    }

    void Object3DPerfPublisher::Publish(LidarObjDet3DPerfData& objects3d_performance_ptr_output, double time){
        ros::Time ros_time((float)time);
        PublishData(objects3d_performance_ptr_output, ros_time);

    }

    void Object3DPerfPublisher::Publish(LidarObjDet3DPerfData& objects3d_performance_ptr_output){
        ros::Time time = ros::Time::now();
        PublishData(objects3d_performance_ptr_output, time);
    }

    void Object3DPerfPublisher::PublishData(LidarObjDet3DPerfData& objects3d_performance_ptr_output, ros::Time time){
        time = ros::Time::now();
    }

    bool Object3DPerfPublisher::HasSubscribers(){
        return publisher_.getNumSubscribers() != 0;
    }
}