/*
*/

#include "lidar_perception/publisher/object3dPublisher.hpp"
#include "glog/logging.h"

namespace lidar_perception {
    Object3DPublisher::Object3DPublisher(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         std::string frame_id,
                                         size_t buff_size)
        :nh_(nh), frame_id_(frame_id) {
        publisher_ = nh_.advertise<lidar_perception_msgs::BoundingBoxes3D>(topic_name, buff_size);
    }

    void Object3DPublisher::Publish(LidarObjects3DData& objects3d_ptr_output, double time){
        ros::Time ros_time((float)time);
        PublishData(objects3d_ptr_output, ros_time);

    }

    void Object3DPublisher::Publish(LidarObjects3DData& objects3d_ptr_output){
        ros::Time time = ros::Time::now();
        PublishData(objects3d_ptr_output, time);
    }

    void Object3DPublisher::PublishData(LidarObjects3DData& objects3d_ptr_output, ros::Time time){
        lidar_perception_msgs::BoundingBoxes3DPtr objects3d_msg_ptr_output(new lidar_perception_msgs::BoundingBoxes3D());
        // for (auto object3d : objects3d_ptr_output.objects3d_ptr->objects3d){
        //     std::cout << object3d.x << " " << object3d.y << " " << object3d.z << std::endl;
        // }

        objects3d_msg_ptr_output->header.stamp = time;
        objects3d_msg_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*objects3d_msg_ptr_output);
    }

    bool Object3DPublisher::HasSubscribers(){
        return publisher_.getNumSubscribers() != 0;
    }
}