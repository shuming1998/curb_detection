#pragma once

#include "../base/pointxyzirt.h"

typedef pcl::PointXYZIRT Point4DIRT;

namespace perception {
namespace curb {

class BaseCurbExtract {
public:
    BaseCurbExtract() = default;
    virtual ~BaseCurbExtract() = default;

    virtual bool Init() = 0;
    virtual bool Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                            pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb) = 0;
};

}  // namespace curb
}  // namespace perception
