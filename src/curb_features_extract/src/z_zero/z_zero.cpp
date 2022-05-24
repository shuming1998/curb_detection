#include "z_zero.h"
#define DEBUG false

namespace perception {
namespace curb {

/// @brief 获得当前UTC时间/秒
inline double getTime_s() {
    auto time_now = std::chrono::system_clock::now();
    auto duration_in_ms = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch());
    return duration_in_ms.count() / 1000000.0;
}

bool ZZero::Init() {}

void ZZero::zZeroMethod()
{
    int p2, p3;
    float alpha, va1, va2, vb1, vb2, max1, max2, d, bracket;
    for (int i = 0; i < params_.scan_num; i++)
    {
        for (int j = params_.curbPoints; j < (ringPointsArray[i].size() - 1) - params_.curbPoints; j++)
        {
            d = sqrt(pow(ringPointsArray[i][j+params_.curbPoints].p.x - ringPointsArray[i][j-params_.curbPoints].p.x, 2) +
                     pow(ringPointsArray[i][j+params_.curbPoints].p.y - ringPointsArray[i][j-params_.curbPoints].p.y, 2));

            //set the distance to be less than 5 meters
            if (d < 5.0000)
            {
                //initialization
                max1 = max2 = abs(ringPointsArray[i][j].p.z);
                va1 = va2 = vb1 = vb2 = 0;

                //initializing vector 'a' and maximal height
                for (int k = j - 1; k >= j - params_.curbPoints; k--)
                {
                    va1 = va1 + (ringPointsArray[i][k].p.x - ringPointsArray[i][j].p.x);
                    va2 = va2 + (ringPointsArray[i][k].p.y - ringPointsArray[i][j].p.y);
                    if (abs(ringPointsArray[i][k].p.z) > max1)
                        max1 = abs(ringPointsArray[i][k].p.z);
                }

                //initializing vector 'b' and maximal height
                for (int k = j + 1; k <= j + params_.curbPoints; k++)
                {
                    vb1 = vb1 + (ringPointsArray[i][k].p.x - ringPointsArray[i][j].p.x );
                    vb2 = vb2 + (ringPointsArray[i][k].p.y  - ringPointsArray[i][j].p.y );
                    if (abs(ringPointsArray[i][k].p.z ) > max2)
                        max2 = abs(ringPointsArray[i][k].p.z);
                }

                va1 = (1 / (float)params_.curbPoints) * va1;
                va2 = (1 / (float)params_.curbPoints) * va2;
                vb1 = (1 / (float)params_.curbPoints) * vb1;
                vb2 = (1 / (float)params_.curbPoints) * vb2;

                bracket = (va1 * vb1 + va2 * vb2) / (sqrt(pow(va1, 2) + pow(va2, 2)) * sqrt(pow(vb1, 2) + pow(vb2, 2)));
                if (bracket < -1)
                    bracket = -1;
                else if (bracket > 1)
                    bracket = 1;

                alpha = acos(bracket) * 180 / M_PI;

                //condition and assignment to group
                if (alpha <= params_.angleFilter2 &&
                   (max1 - abs(ringPointsArray[i][j].p.z ) >= params_.curbHeight ||
                    max2 - abs(ringPointsArray[i][j].p.z) >= params_.curbHeight) &&
                    abs(max1 - max2) >= 0.05)
                {
                    ringPointsArray[i][j].isCurbPoint = true;
                }
            }
        }
    }
}

// 容器填充 starMethod:pointsArray; xZeroMethod and zZeroMethod:ringPointsArray
void ZZero::fillPointsArray(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in)
{
    pointsArray.clear();
    ringPointsArray.clear();
    pointsArray.resize(pts_in.size());
    ringPointsArray.resize(params_.scan_num, std::vector<Point>(params_.horizon_num));
    size_t piece = pts_in.size();
    float bracket = 0;
    for (int i = 0; i < piece; i++)
    {
        if (pts_in.points[i].ring != 0 && pts_in.points[i].x > 3)
        {
            //fill PointsArray
            pointsArray[i].p = pts_in.points[i];
            pointsArray[i].d = sqrt(pow(pointsArray[i].p.x, 2) + pow(pointsArray[i].p.y, 2) + pow(pointsArray[i].p.z, 2));
            pointsArray[i].id = i;
            pointsArray[i].isCurbPoint = false;
            bracket = abs(pointsArray[i].p.z) / pointsArray[i].d;
            /*required because of rounding errors*/
            if (bracket < -1) bracket = -1;
            else if (bracket > 1) bracket = 1;
            /*calculation and conversion to degrees*/
            if (pointsArray[i].p.z < 0)
            {
                //计算反余弦，并转化为角度
                pointsArray[i].alpha = acos(bracket) * RAD;
            }
            else
            {
                //计算反正弦，并转化为角度
                pointsArray[i].alpha = (asin(bracket) * RAD) + 90;
            }
            
            //fill ringPointsArray
            int id = (params_.lidar_half_ang - atan2(pts_in.points[i].y, pts_in.points[i].x) * RAD)\
             / params_.horizon_ang;
            if (id > params_.horizon_num - 1)
            {
                continue;
            }
            float d_ = 0;  //sqrt(pow(pointsArray[i].p.x, 2) + pow(pointsArray[i].p.y, 2));
            float theta = atan2(pts_in.points[i].y, pts_in.points[i].x);
            //ringPointsArray[pts_in.points[i].ring - 1].push_back(Point{pts_in.points[i], d_, false, i, theta});
            ringPointsArray[pts_in.points[i].ring - 1][id] = Point{pts_in.points[i], d_, false, i, theta};
        }
    }
}

void ZZero::curbPtsCollect(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in, 
                           pcl::PointCloud<pcl::PointXYZIRT>& pts_curb)
{
    std::vector<bool> ptsType(pts_in.size(), false);
    
    for(const auto xZero : ringPointsArray)
    {
        for (const auto p : xZero)
        {
            if (p.isCurbPoint == true)
            {
                ptsType[p.id] = true;
            }
        }
    }

    for(const auto p : pointsArray)
    {
        if (ptsType[p.id]) 
        {
            pts_curb.push_back(pts_in.points[p.id]);
        }
    }
}

bool ZZero::Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                       pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb)
{
    double step_0 = getTime_s();
    std::cout << 0 << '\n';
    this->fillPointsArray(cloud_in);
    double step_1 = getTime_s();
    std::cout << 1 << '\n';
    this->zZeroMethod();
    double step_2 = getTime_s();
    std::cout << 2 << '\n';
    this->curbPtsCollect(cloud_in, cloud_curb);
    double step_3 = getTime_s();
    std::cout << 3 << '\n';
    std::cout << "zZero time :\n" 
              << "fillPointsArray : " << (step_1 - step_0) * 1000.0 << " ms,\n"
              << "zZeroMethod : " << (step_2 - step_1) * 1000.0 << " ms, \n"
              << "curbPtsCollect : " << (step_3 - step_2) * 1000.0 << " ms, \n";
}

}  // namespace curb
}  // namespace perception
