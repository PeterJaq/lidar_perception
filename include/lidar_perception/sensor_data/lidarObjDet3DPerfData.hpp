#ifndef LIDAR_PERCEPTION_SENSOR_DATA_OBJDET3DPERFDATA_HPP_
#define LIDAR_PERCEPTION_SENSOR_DATA_OBJDET3DPERFDATA_HPP_

#include <deque>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_perception {

struct DetectorPerformance
    {
        /* data */
        double timecost_max = 0.0;

};

class LidarObjDet3DPerfData {
  public:
    // using POINT = pcl::PointXYZI;'

    double timecost = 0.0;
    // std::vector<Object3D> objects3d;
    // using OBJECT = object;
    // using OBJECTS = std::vector<OBJECT>;
    // using OBJECTS_PTR = OBJECTS *vec_ptr;
    LidarObjDet3DPerfData()
      :det_perf (new DetectorPerformance) {
    }

    public:
      DetectorPerformance *det_perf;
};
}

#endif