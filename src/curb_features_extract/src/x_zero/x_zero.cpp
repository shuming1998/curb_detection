#include "x_zero.h"
#define DEBUG false

namespace perception {
namespace curb {

/// @brief 获得当前UTC时间/秒
inline double getTime_s() {
    auto time_now = std::chrono::system_clock::now();
    auto duration_in_ms = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch());
    return duration_in_ms.count() / 1000000.0;
}

bool XZero::Init() {}

// 容器填充 starMethod:pointsArray; xZeroMethod and zZeroMethod:ringPointsArray
void XZero::fillPointsArray(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in)
{
    pointsArray.clear();
    ringPointsArray.clear();
    pointsArray.resize(pts_in.size());
    ringPointsArray.resize(params_.scan_num, std::vector<Point>(params_.horizon_num));

    size_t piece = pts_in.size();
    float bracket = 0;
    for (int i = 0; i < piece; ++i)
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
            float d_ = 0;//sqrt(pow(pointsArray[i].p.x, 2) + pow(pointsArray[i].p.y, 2));
            float theta = atan2(pts_in.points[i].y, pts_in.points[i].x);
            //ringPointsArray[pts_in.points[i].ring - 1].push_back(Point{pts_in.points[i], d_, false, i, theta});
            ringPointsArray[pts_in.points[i].ring - 1][id] = Point{pts_in.points[i], d_, false, i, theta};
        }
    }
}

void XZero::xZeroMethod()
{
    int p2, p3;
    float alpha, x1, x2, x3, d, bracket;
    for (int i = 0; i < params_.scan_num; ++i)
    {
        //assigning new Y values while keeping X = 0
        for (int j = 1; j <= (ringPointsArray[i].size() - 1); ++j)
        {
            ringPointsArray[i][j].newY = ringPointsArray[i][j-1].newY + 0.0100;
        }

        //evaluation of the points in an arc - x-zero method
        for (int j = params_.curbPoints; j < (ringPointsArray[i].size() - params_.curbPoints - 1); ++j)
        {
            p2 = j + params_.curbPoints / 2;
            p3 = j + params_.curbPoints;

            d = sqrt(pow(ringPointsArray[i][p3].p.x - ringPointsArray[i][j].p.x , 2) +
                     pow(ringPointsArray[i][p3].p.y  - ringPointsArray[i][j].p.y , 2));

            //set the distance to be less than 5 meters
            if (d < 5.0000)
            {
                x1 = sqrt(pow(ringPointsArray[i][p2].newY  - ringPointsArray[i][j].newY , 2) +
                          pow(ringPointsArray[i][p2].p.z - ringPointsArray[i][j].p.z , 2));
                x2 = sqrt(pow(ringPointsArray[i][p3].newY - ringPointsArray[i][p2].newY, 2) +
                          pow(ringPointsArray[i][p3].p.z  - ringPointsArray[i][p2].p.z , 2));
                x3 = sqrt(pow(ringPointsArray[i][p3].newY  - ringPointsArray[i][j].newY , 2) +
                          pow(ringPointsArray[i][p3].p.z  - ringPointsArray[i][j].p.z , 2));

                bracket = (pow(x3, 2) - pow(x1, 2) - pow(x2, 2)) / (-2 * x1 * x2);
                if (bracket < -1)
                    bracket = -1;
                else if (bracket > 1)
                    bracket = 1;

                alpha = acos(bracket) * RAD;

                //condition and assignment to group
                if (alpha <= params_.angleFilter1 &&
                    (abs(ringPointsArray[i][j].p.z  - ringPointsArray[i][p2].p.z ) >= params_.curbHeight ||
                    abs(ringPointsArray[i][p3].p.z  - ringPointsArray[i][p2].p.z ) >= params_.curbHeight) &&
                    abs(ringPointsArray[i][j].p.z  - ringPointsArray[i][p3].p.z ) >= 0.05)
                {
                    ringPointsArray[i][p2].isCurbPoint  = true;
                }
            }
        }
    }
}

void XZero::curbPtsCollect(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in, 
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

    //int nums = 0;
    for(const auto p : pointsArray)
    {
        if (ptsType[p.id]) 
        {
            pts_curb.push_back(pts_in.points[p.id]);
        }
        //++nums;
    }
}

bool XZero::Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                       pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb) 
{
    double step_0 = getTime_s();
    std::cout << 0 << '\n';
    
    this->fillPointsArray(cloud_in);
    double step_1 = getTime_s();
    
    std::cout << 1 << '\n';
    
    this->xZeroMethod();
    double step_2 = getTime_s();
    
    std::cout << 2 << '\n';
    
    this->curbPtsCollect(cloud_in, cloud_curb);
    double step_3 = getTime_s();
    
    std::cout << 3 << '\n';
    
    std::cout << "XZero time :\n" 
              << "fillPointsArray : " << (step_1 - step_0) * 1000.0 << " ms,\n"
              << "xZeroMethod : " << (step_2 - step_1) * 1000.0 << " ms, \n"
              << "curbPtsCollect : " << (step_3 - step_2) * 1000.0 << " ms, \n";
}


}  // namespace curb
}  // namespace perception
