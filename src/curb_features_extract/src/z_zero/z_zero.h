#ifndef Z_ZERO_H
#define Z_ZERO_H

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
// opencv
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
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
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/range_image/range_image.h>

#include "../interface/interface_features_extract.h"

#define PI 3.1415926
#define RAD 57.295780
#define PER_RAD 1/RAD

namespace perception {
namespace curb {

namespace params {
    const Eigen::Vector3d lidar_pos{5.4, -0.2, 3.2};
    const Eigen::Vector3d lidar_ori{0.8, 5.2, 0.2};
}

struct ZZeroParams {
    //lidar params
    const float horizon_range_ang = 128.1;
    const int scan_num = 128;
    const int horizon_num = 1281;
    const float horizon_ang = 0.1;
    //const float vertical_ang = 0.2;
    const float lidar_half_ang = horizon_range_ang / 2;
    const float lidar_half_rad = lidar_half_ang * PER_RAD;

    //method params
    const int rep = 128;
    const float width = 0.2;
    const float angleFilter2 = 100; /*Z0_method 140*/
    const int curbPoints = 5;
    const float curbHeight = 0.05; //(0.01, 0.50)
};

struct Point
{
    Point4DIRT p;
    float d;
    float alpha;
    bool isCurbPoint = 0;
    int id;
    float theta;
    float newY;
    Point() = default;
    ~Point() = default;
    Point(Point4DIRT p_, float d_, bool isCurbPoint_, int id_, float theta_) : p(p_), d(d_), isCurbPoint(isCurbPoint_), id(id_), theta(theta_){}
};

struct polar //polar-coordinate struct for the points
{
    int id;
    float r;
    float fi;
};

struct box  //beam
{
    std::vector<polar> p;
    box *l, *r;
    bool yx;
    float o, d;
};

class ZZero: public BaseCurbExtract {
public:
    ZZero() = default;
    ~ZZero() = default;
    bool Init() override;
    bool Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                    pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb) override;

private:
    void fillPointsArray(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in);
    void zZeroMethod();
    void curbPtsCollect(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in, 
                        pcl::PointCloud<pcl::PointXYZIRT>& pts_curb);

private:
    ZZeroParams params_;
    std::vector<Point> pointsArray;
    std::vector<std::vector<Point> > ringPointsArray;
};


}  // namespace curb
}  // namespace perception
#endif
