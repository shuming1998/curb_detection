#include "star_shaped.h"
#define DEBUG false

namespace perception {
namespace curb {

std::vector<box> beams(params::rep);
std::vector<box*> beamp(params::rep + 1);

// 按极坐标半径排序
inline bool ptcmpr(polar &a, polar &b) { return (a.r < b.r); }
// 按正切值排序
inline bool pthcmpr(Point &a, Point &b) { return (a.theta < b.theta); }
inline float slope(float x0, float y0, float x1, float y1) { return (y1 - y0) / (x1 - x0); }

/// @brief 获得当前UTC时间/秒
inline double getTime_s() {
    auto time_now = std::chrono::system_clock::now();
    auto duration_in_ms = std::chrono::duration_cast<std::chrono::microseconds>(time_now.time_since_epoch());
    return duration_in_ms.count() / 1000000.0;
}

// beam初始化
void StarShaped::beamInit()
{
    {
        // fi:弧度
        float fi, off = 0.5 * params_.width;
        for (int i = 0; i < params_.rep; i++)
        {
            // 第 i 个 beam 夹角为i°，化为弧度 fi
            fi = (i - params_.lidar_half_ang) * PER_RAD;
            //std::cout << i << " th tan(fi): " << tan(fi) << '\n';
            // 靠近 Y 轴
            if (abs(tan(fi)) > 1)
            {
                beams[i].yx = true;
                //靠近 y 轴，d 是正切值的倒数 1/tan(fi)
                beams[i].d = tan(0.5 * M_PI - fi);
                // o 是beam的半宽度在 X 轴上的投影长度
                beams[i].o = abs(off / sin(fi));
                //std::cout << "Y : the " << i << "th beam's length is " <<  beams[i].o << '\n';
            }
            // 靠近 X 轴
            else
            {
                beams[i].yx = false;
                beams[i].d = tan(fi);
                beams[i].o = abs(off / cos(fi));
                //std::cout << "X : the " << i << "th beam's length is " <<  beams[i].o << '\n';
            }
            beamp[i] = &beams[i]; //初始化 beam 指针
        }
    }
}

bool StarShaped::Init() {
    this->beamInit();
}

// 容器填充 starMethod:pointsArray; xZeroMethod and zZeroMethod:ringPointsArray
void StarShaped::fillPointsArray(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in)
{
    pointsArray.clear();
    pointsArray.resize(pts_in.size());
    size_t piece = pts_in.size();
    float bracket = 0;

    for (int i = 0; i < piece; i++)
    {
        if (pts_in.points[i].ring != 0 && pts_in.points[i].x > 3)
        {
            //------------------------------fill 2D array
            /*--- filling the first 4 columns ---*/
            pointsArray[i].p = pts_in.points[i];
            pointsArray[i].d = sqrt(pow(pointsArray[i].p.x, 2) + pow(pointsArray[i].p.y, 2) + pow(pointsArray[i].p.z, 2));
            pointsArray[i].id = i;
            pointsArray[i].isCurbPoint = false;
            /*--- filling the 5. column ---*/
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
        }
    } 
}

void StarShaped::beamfunc(const int& tid, std::vector<Point> &pointsArray)
{
    int j = 0, s = beams[tid].p.size();
    //std::cout << tid << ": " << s << '\n';
    float c;
    
    double step_3 = getTime_s();
    if (s > 10000)
    {
        return;
    }
    // 如果 beam 靠近 Y 轴，根据点的 x 坐标来判断是否在 beam 中
    if (beams[tid].yx)
    {
        // s 是 beam 中点的个数，遍历每个点，剔除 beam 之外的点
        while (j < s)
        {
            //靠近Y轴的beam，d是正切值的倒数，d乘以点的y坐标，得到在X轴上的c值
            //如果点的 x 坐标落在以c为中心、以 beam 的宽度在 X 轴上的投影 o 为半径的区间内，该点就是真的属于 beam 范围内的点
            c = abs(beams[tid].d * pointsArray[beams[tid].p[j].id].p.y);
            if ((c - beams[tid].o) < pointsArray[beams[tid].p[j].id].p.x < (c + beams[tid].o))
            {
            	++j;
            }
            else
            {
                //移除该点
                beams[tid].p.erase(beams[tid].p.begin() + j);
                --s;
            }
        }
    }
    // 如果靠近 X 轴，则根据 Y 坐标来判断
    else
    {
        while (j < s)
        {
            c = abs(beams[tid].d * pointsArray[beams[tid].p[j].id].p.x);
            if ((c - beams[tid].o) < pointsArray[beams[tid].p[j].id].p.y < (c + beams[tid].o))
            {
                ++j;
            }
            else
            {
                beams[tid].p.erase(beams[tid].p.begin() + j);
                --s;
            }
        }
    }
    
    double step_4 = getTime_s();
    //std::cout << "func1-remove_pt: " << (step_4 - step_3) * 1000 << " ms\n";
    
    //迭代前 先把点按照极坐标长度排序
    std::sort(beams[tid].p.begin(), beams[tid].p.end(), [&](polar &a, polar &b){ return a.r < b.r; });
    //std::sort(beams[tid].p.begin(), beams[tid].p.end(), ptcmpr);
    
    double step_5 = getTime_s();
    //std::cout << "func2-sort_time: " << (step_5 - step_4) * 1000 << " ms\n";
    {
        // 开始迭代计算每个点是否为候选的路沿点，只有点的个数大于1才计算
        if (s > 1)
        {
            // 从第 dmin 个点以后，才加入第二个判断条件
            int dmin = params_.dmin;
            float kdev = params_.kdev;
            float kdist = params_.kdist;
            // avg:局部平均斜率 dev:局部平均斜率的局部平均标准差 nan:nan值斜率点的个数
            float avg = 0, dev = 0, nan = 0;
            float ax, ay, bx, by, slp;
            // bx初始值为第一个点的极坐标半径
            bx = beams[tid].p[0].r;
            // by初始值为第一个点的高度坐标z
            by = pointsArray[beams[tid].p[0].id].p.z;
			// 开始迭代(找到路沿点就跳出循环)
            for (int i = 1; i < s; i++) 
            {      
                // 每次更新一个点，计算新点和上一个点的斜率等值
                ax = bx;
                bx = beams[tid].p[i].r;
                ay = by;
                by = pointsArray[beams[tid].p[i].id].p.z;
                //计算相邻两个点的斜率 (by - ay) / (bx - ax)
                slp = slope(ax, ay, bx, by);
                //std::cout << "斜率slp = " << abs(slp) << '\n';
                // 过滤 nan 值
                if (isnan(slp))
                {
                    nan++;
                }
                // 迭代计算
                else
                {
                    // avg是 每个点和上一个点的斜率的 局部均值
                    // 每加入一个点就更新一次，所以说是局部
                    avg *= i - nan - 1;
                    avg += slp;
                    avg *= 1 / (i - nan);
                    // dev是 当前斜率和avg的 局部标准差的 局部均值
                    // 每加入一个点就更新一次，所以说是局部
                    dev *= i - nan - 1;
                    dev += abs(slp - avg);
                    dev *= 1 / (i - nan);
                }
                // 如果斜率大于阈值
                // 或者对于第i个点之后的点,根据高度差和斜率加权计算出的值大于当前dev
                if ((i > dmin && (slp * slp - avg * avg) * kdev * ((bx - ax) * kdist) > dev))//slp > params::slope_param ||
                {
                    // 该点是候选路沿点，找到第一个候选路沿点就跳出遍历
                    // 因为目的是找路沿的边界点，之后的就不是边界点了，对路面无影响
                    pointsArray[beams[tid].p[i].id].isCurbPoint = true;
                    break;
                }
            }
        }
    }
    double step_6 = getTime_s();
    //std::cout << "func3-find_curb: " << (step_6 - step_5) * 1000 << " ms\n";
    beams[tid].p.clear();
    double step_7 = getTime_s();
    //std::cout << "func4-clear: " << (step_7 - step_6) * 1000 << " ms\n";
}

void StarShaped::starShapedSearch()
{	
    double step_0 = getTime_s();
    //beamp是 beam 的指针数组，对应一组 beams 中的每个 beam
    beamp.push_back(&beams[0]);
    //f 点对应 beam 的索引; s是输入点云的尺寸
    int f, s = pointsArray.size();
    // 极坐标半径和角度
    float r, fi;
    int b_id;
    // 把所有点放入对应的beam，记录极坐标和id
    for (int i = 0; i < s; ++i)
    {
        // 点在极坐标下的半径
        r = sqrt(pointsArray[i].p.x * pointsArray[i].p.x + pointsArray[i].p.y * pointsArray[i].p.y);
        // 点的极坐标角度 (-pi, pi)
        fi = atan2(pointsArray[i].p.y, pointsArray[i].p.x);
        b_id = pointsArray[i].id;
        // 确保 fi 范围是(0, PI) 
        //fi += params_.lidar_half_rad;     
        // 计算点所属的 beam 在 beams 中的索引
        f = (int)((fi * RAD) + params_.lidar_half_ang);
        //std::cout << i << " : " << f << "\n";
        // 根据索引，把该点放入对应的 beam 中
        if (f >= 0 && f < params_.rep)
        {
            beamp[f]->p.push_back(polar{b_id, r, fi});
            //std::cout << "beam " << f <<" has " << beamp[f]->p.size() <<  " points" << std::endl;
        }
    }
    // 防止对同一块内存多次释放
    beamp.pop_back();//removing pointer (to prevent "double free" error)
    
    double step_1 = getTime_s();
    
    //std::cout << "beam_step_1: " << (step_1 - step_0) * 1000 << " ms\n";
    // 点云分配完成，处理每一个 beam
    //std::cout << "rep_is: " << params_.rep << std::endl;
    for (int i = 1; i < params_.rep; ++i)   //for every beam...
    {
        double inside_1 = getTime_s();
        // 迭代计算 beam 中的点的核心函数
        beamfunc(i, pointsArray);
        double inside_2 = getTime_s();
        //std::cout << "run at " << i << " times use: " << (inside_2 - inside_1) * 1000 << " ms\n";
    }

    double step_2 = getTime_s();
    //std::cout << "beam_step_2: " << (step_2 - step_1) * 1000 << " ms\n";
}

void StarShaped::curbPtsCollect(const pcl::PointCloud<pcl::PointXYZIRT>& pts_in, 
                                pcl::PointCloud<pcl::PointXYZIRT>& pts_curb)
{
    std::vector<bool> ptsType(pts_in.size(), false);
    
    for(const auto star : pointsArray)
    {
        if (star.isCurbPoint)
        {
            ptsType[star.id] = true;
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

bool StarShaped::Processing(const pcl::PointCloud<pcl::PointXYZIRT>& cloud_in,
                            pcl::PointCloud<pcl::PointXYZIRT>& cloud_curb) 
{
    double step_0 = getTime_s();
    
    this->fillPointsArray(cloud_in);
    double step_1 = getTime_s();
    
    this->starShapedSearch();
    double step_2 = getTime_s();
    
    this->curbPtsCollect(cloud_in, cloud_curb);
    double step_3 = getTime_s();

    std::cout << "star shaped time :\n" 
              << "fillPointsArray : " << (step_1 - step_0) * 1000.0 << " ms,\n"
              << "starShapedSearch : " << (step_2 - step_1) * 1000.0 << " ms, \n"
              << "curbPtsCollect : " << (step_3 - step_2) * 1000.0 << " ms, \n";
}

}  // namespace curb
}  // namespace perception
