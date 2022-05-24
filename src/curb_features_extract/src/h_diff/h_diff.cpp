#include "h_diff.h"
#define DEBUG false

namespace perception {
namespace curb {

/// @brief 获得当前UTC时间/秒
inline double getTime_s() {
    auto time_now = std::chrono::system_clock::now();
    auto duration_in_ms = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch());
    return duration_in_ms.count() / 1000000.0;
}

/*
inline float max_zdiff(const std::vector<pcl::PointXYZIRT> &data, size_t start, size_t end, bool increase = true)
{
    if (!increase)
    {
        size_t tmp = start;
        start = end;
        end = tmp;
    }
    float max = data[start].z;
    float min = data[start].z;
    float z;
    for (size_t i = start; i < end; i++)
    {
        z = data[i].z;
        if (z > max)
        {
            max = z;
        }
        if (z < min && z != 0)
        {
            min = z;
        }
    }
    return max - min;
}
*/

bool HDiff::Init() {}

void HDiff::classifyPointsByRing(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in, 
                                std::vector<std::vector<pcl::PointXYZIRT> >& ringPointsArrays)
{
    ringPointsArrays.clear();
    ringPointsArrays.resize(params_.scan_num, std::vector<pcl::PointXYZIRT>(params_.horizon_num));
    
    for (const auto& pt : pts_in)
    {
        if (pt.ring != 0 && pt.x > 1)
        {
            int id = (params_.lidar_half_ang - atan2(pt.y, pt.x) * RAD) / params_.horizon_ang;
            if (id >= params_.horizon_num - 1 || pt.ring >= params_.scan_num - 1)
            {
                continue;
            }
            ringPointsArrays[pt.ring - 1][id - 1] = pt;
        }
    }
}

float max_h_diff(const std::vector<pcl::PointXYZIRT>& ring_data, int i, int windowSize)
{
    float min_h = FLT_MAX;
    float max_h = -FLT_MAX;
    for(int j = i; j < i + windowSize; ++j)
    {
        if (ring_data[j].z != 0)
        {
            min_h = std::min(min_h, ring_data[j].z);
            max_h = std::max(max_h, ring_data[j].z);
        }
    }
    return std::fabs(max_h - min_h);
}

// 每条ring上按照高度差提取curb
std::vector<pcl::PointXYZIRT> HDiff::detectCurb(const std::vector<pcl::PointXYZIRT>& ring_data)
{
    std::vector<pcl::PointXYZIRT> out;
    out.reserve(ring_data.size());

    for (int i = 0; i < (ring_data.size() - params_.windowSize); i += params_.windowSize)
    {
        float h_diff = max_h_diff(ring_data, i, params_.windowSize);
        //std::cout << "h_diff: " << h_diff << '\n';
        if (std::fabs(h_diff) > params_.HDiffThreshold && std::fabs(h_diff) < params_.HDiffThresholdMax)
        {
            out.insert(out.end(), ring_data.begin() + i, ring_data.begin() + i + params_.windowSize);
        }
    }
    return out;
}
/*
std::vector<pcl::PointXYZIRT> HDiff::detectCurb(const std::vector<pcl::PointXYZIRT>& ring_data)
{
    std::vector<pcl::PointXYZIRT> out;
    out.reserve(ring_data.size());

    // 是否匹配成功
    // has_curb z轴高度差
    bool has_curb = false; 
    // 从中心向左移动的窗格
    for (auto it = ring_data.begin(); it != ring_data.end(); ++it)
    {
        // 检测高度差
        if (!has_curb)
        {
            size_t start = it - ring_data.begin();
            size_t end = it + params_.windowSize - ring_data.begin();

            float zdiff = max_zdiff(ring_data, start, end);
            if (std::abs(zdiff) > params_.HDiffThreshold)
            {
                out.insert(out.end(), it, it + params_.windowSize);
                has_curb = true;
            }
        }
        if (has_curb)
        {
            break;
        }
    }
    has_curb = false;
    // 从中心向右移动的窗格
    for (auto it = ring_data.rbegin(); it != ring_data.rend(); ++it)
    {
        // 检测高度差
        if (!has_curb)
        {
            size_t start = (it + params_.windowSize).base() - ring_data.begin();
            size_t end = it.base() - ring_data.begin();

            float zdiff = max_zdiff(ring_data, start, end);
            if (std::abs(zdiff) > params_.HDiffThreshold)
            {
                // base方法拿到反向迭代器对应的正向迭代器
                out.insert(out.end(), (it + params_.windowSize - 1).base(), it.base());
                has_curb = true;
            }
        }
        if (has_curb)
        {
            break;
        }
    }
    return out;
}
*/

void HDiff::HDiffMethod(std::vector<std::vector<pcl::PointXYZIRT> >& ringPointsArrays, 
                        pcl::PointCloud<pcl::PointXYZIRT>& pts_curb)
{
    std::vector<pcl::PointXYZIRT> all_curbs;
    for (size_t i = 0; i < ringPointsArrays.size(); i++)
    {
        auto curbs = detectCurb(ringPointsArrays[i]);
        all_curbs.reserve(all_curbs.size() + curbs.size());
        all_curbs.insert(all_curbs.end(), curbs.begin(), curbs.end());
    }
    pts_curb.reserve(all_curbs.size() + 1);
    for (auto& pt_curb : all_curbs)
    {
        if (pt_curb.ring != 0)
        {
            pts_curb.push_back(pt_curb);
        }
    }
}

bool HDiff::Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                       pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb)
{
    std::vector<std::vector<pcl::PointXYZIRT> > ringPointsArrays;

    double step_0 = getTime_s();
    std::cout << 0 << '\n';
    this->classifyPointsByRing(cloud_in, ringPointsArrays);
    double step_1 = getTime_s();
    std::cout << 1 << '\n';
    this->HDiffMethod(ringPointsArrays, cloud_curb);
    double step_2 = getTime_s();
    std::cout << 2 << '\n';

    std::cout << "HDiff time :\n" 
              << "fillPointsArrays : " << (step_1 - step_0) * 1000.0 << " ms,\n"
              << "HDiffMethod : " << (step_2 - step_1) * 1000.0 << " ms, \n";
}

}  // namespace curb
}  // namespace perception
