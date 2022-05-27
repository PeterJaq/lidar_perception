#ifndef LIDAR_PERCEPTION_PUBLISHER_OBJECT3D_PUBLISHER_HPP
#define LIDAR_PERCEPTION_PUBLISHER_OBJECT3D_PUBLISHER_HPP

#include "lidar_perception_msgs/BoundingBoxes3D.h"
// #include ""
namespace lidar_perception
{
    class Object3DPublisher{
        public:
            Object3DPublisher(ros::NodeHandel& nh,
                            std::string topic_name,
                            std::string frame_id,
                            size_t buff_size);
            Object3DPublisher() = default;

            void publish(lidar_perception_msgs::BoundingBoxs3D objects3d_ptr_input, double time);
            void publish(lidar_perception_msgs::BoundingBoxs3D objects3d_ptr_input);

            bool HasSubscribers(lidar_perception_msgs::BoundingBoxs3D objects3d_ptr_input, ros::Time time);

        private:
            void PublishData();
        private:
            ros::NodeHandle nh_;
            ros::Publisher publisher_;
            std::string frame_id;
    }
} // namespace lidar_perception

#endif 