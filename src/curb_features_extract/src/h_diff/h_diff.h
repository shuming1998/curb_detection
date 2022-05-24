#ifndef Z_ZERO_H
#define Z_ZERO_H

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// base
#include <chrono>
#include <stdio.h>
#include <memory>
#include <stdint.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <string>
// pcl
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "../interface/interface_features_extract.h"

#define PI 3.1415926
#define RAD 57.295780
#define PER_RAD 1/RAD

namespace perception {
namespace curb {

namespace params {
    const Eigen::Vector3d lidar_pos{5.4, -0.2, 3.2};
    const Eigen::Vector3d lidar_ori{0.8, 5.2, 0.2};
    const std::string topic = "/hesai/pandar_points";
}

struct HDiffParams {

    const float horizon_ang = 0.1;
    const int scan_num = 128;
    const int horizon_num = 1281;
    const float horizon_range_ang = 128.1;
    const float lidar_half_ang = horizon_range_ang * 0.5;

    const int windowSize = 12;
    const float HDiffThreshold = 0.07;
    const float HDiffThresholdMax = 0.25;
};


class HDiff: public BaseCurbExtract {
public:
    HDiff() = default;
    ~HDiff() = default;
    bool Init() override;
    bool Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                    pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb) override;

private:
    void classifyPointsByRing(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in, 
                            std::vector<std::vector<pcl::PointXYZIRT> >& ringPointsArrays);
    void HDiffMethod(std::vector<std::vector<pcl::PointXYZIRT> >& ringPointsArrays, 
                     pcl::PointCloud<pcl::PointXYZIRT>& pts_curb);
    std::vector<pcl::PointXYZIRT> detectCurb(const std::vector<pcl::PointXYZIRT>& ring_data);
private:
    HDiffParams params_;
    //std::vector<pcl::PointXYZIRT> ringPointsArray;
    //std::vector<std::vector<pcl::PointXYZIRT> > ringPointsArrays;
};


}  // namespace curb
}  // namespace perception
#endif
