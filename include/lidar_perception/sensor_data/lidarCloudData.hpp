#ifndef LIDAR_PERCEPTION_SENSOR_DATA_LIDARCLOUDDATA_HPP_
#define LIDAR_PERCEPTION_SENSOR_DATA_LIDARCLOUDDATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lidar_localization {
class CloudData {
  public:
    using POINT = pcl::PointXYZI;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

  public:
    CloudData()
      :cloud_ptr(new CLOUD()) {
    }

  public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}

#endif