#include "lidar_perception/publisher/object3dVisPublisher.hpp"
#include "glog/logging.h"

namespace lidar_perception {
    Object3DVisPublisher::Object3DVisPublisher(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         std::string frame_id,
                                         size_t buff_size)
        :nh_(nh), frame_id_(frame_id) {
        publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(topic_name, buff_size);
    }

    void Object3DVisPublisher::Publish(LidarObjects3DData& objects3d_ptr_output, double time){
        ros::Time ros_time((float)time);
        PublishData(objects3d_ptr_output, ros_time);

    }

    void Object3DVisPublisher::Publish(LidarObjects3DData& objects3d_ptr_output){
        ros::Time time = ros::Time::now();
        PublishData(objects3d_ptr_output, time);
    }

    void Object3DVisPublisher::PublishData(LidarObjects3DData& objects3d_ptr_output, ros::Time time){
        lidar_perception_msgs::BoundingBoxes3DPtr objects3d_msg_ptr_output(new lidar_perception_msgs::BoundingBoxes3D());
        
        visualization_msgs::MarkerArrayPtr marker_array_ptr (new visualization_msgs::MarkerArray());
        int objId = 0;
        for (auto object3d : objects3d_ptr_output.objects3d_ptr->objects3d){

            visualization_msgs::MarkerPtr bbox_marker_ptr (new visualization_msgs::Marker());
            bbox_marker_ptr->header.frame_id = "odom";
            bbox_marker_ptr->header.stamp = ros::Time::now();
            bbox_marker_ptr->lifetime = ros::Duration();
            bbox_marker_ptr->ns = "";
            bbox_marker_ptr->id = objId;
            bbox_marker_ptr->type = visualization_msgs::Marker::CUBE;
            // color
            bbox_marker_ptr->color.r = 0.0f;
            bbox_marker_ptr->color.g = 1.0f;
            bbox_marker_ptr->color.b = 1.0f;
            bbox_marker_ptr->color.a = 0.5f;

            // pose
            bbox_marker_ptr->pose.position.x = object3d.x;
            bbox_marker_ptr->pose.position.y = object3d.y;
            bbox_marker_ptr->pose.position.z = object3d.z;

            // orientation
            Eigen::Quaternionf quaternion;
            quaternion = Euler2Quaternion(object3d.yaw, object3d.pitch, object3d.roll);
            bbox_marker_ptr->pose.orientation.x = quaternion.x();
            bbox_marker_ptr->pose.orientation.y = quaternion.y();
            bbox_marker_ptr->pose.orientation.z = quaternion.z();
            bbox_marker_ptr->pose.orientation.w = quaternion.w();

            // whl
            bbox_marker_ptr->scale.x = object3d.w;
            bbox_marker_ptr->scale.y = object3d.l;
            bbox_marker_ptr->scale.z = object3d.h;

            marker_array_ptr->markers.emplace_back(*bbox_marker_ptr);

            objId += 1;
        }
        publisher_.publish(*marker_array_ptr);
    }


    bool Object3DVisPublisher::HasSubscribers(){
        return publisher_.getNumSubscribers() != 0;
    }
    
    Eigen::Quaternionf Object3DVisPublisher::Euler2Quaternion(float yaw, float pitch, float roll){
        Eigen::Matrix3f matYaw(3, 3), matRoll(3, 3), matPitch(3, 3), matRotation(3, 3);
        Eigen::Quaternionf q;
        q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX())
            * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
            * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());

        return q;
    }


}