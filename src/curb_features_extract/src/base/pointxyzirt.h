#define PCL_NO_PRECOMPILE  //从PCL-1.7开始，您需要定义PCL_NO_PRECOMPILE，然后才能包含任何PCL头文件来包含模板化算
#ifndef POINTXYZIRT_H
#define POINTXYZIRT_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace pcl {

struct PointXYZIRT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (uint8_t, intensity, intensity)
                                 (uint16_t, ring, ring)
                                 (double, timestamp, timestamp))


#endif
