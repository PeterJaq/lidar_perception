#ifndef LIDAR_PERCEPTION_SENSOR_DATA_LIDAROBJECTS3DDATA_HPP_
#define LIDAR_PERCEPTION_SENSOR_DATA_LIDAROBJECTS3DDATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "lidar_perception_msgs/BoundingBoxes3D.h"

namespace lidar_perception {

struct Object3D
    {
        /* data */
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        double w = 0.0;
        double h = 0.0;
        double l = 0.0;

        double yaw = 0.0;
        double pitch = 0.0;
        double row = 0.0;

        int id = 0.0;
        int cls = 0.0;
        double probability = 0.0;
};

struct Objects3D{
        std::vector<Object3D> objects3d;
        int object_counter = 0;
};

class LidarObjects3DData {
  public:
    // using POINT = pcl::PointXYZI;'

    double time = 0.0;
    Object3D object3d;
    // std::vector<Object3D> objects3d;
    // using OBJECT = object;
    // using OBJECTS = std::vector<OBJECT>;
    // using OBJECTS_PTR = OBJECTS *vec_ptr;
    LidarObjects3DData()
      :objects3d_ptr (new Objects3D) {
    }

    public:
      Objects3D *objects3d_ptr;
};
}

#endif