#include "../base/pointxyzirt.h"
#include "../interface/interface_features_extract.h"
#include "../h_diff/h_diff.h"

// ROS
#include <ros/ros.h>

// 点云位姿转换
void ptsTrans(pcl::PointCloud<Point4DIRT>& untrans_pts_in)
{
    Eigen::AngleAxisd rollAngle = (Eigen::AngleAxisd(perception::curb::params::lidar_ori.x()/RAD, Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle = (Eigen::AngleAxisd(perception::curb::params::lidar_ori.y()/RAD, Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle = (Eigen::AngleAxisd(perception::curb::params::lidar_ori.z()/RAD, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix4d trans_matrix;
    trans_matrix.block<3, 1>(0, 3) = perception::curb::params::lidar_pos;
    trans_matrix.block<3, 3>(0, 0) = rotation_matrix;
    pcl::transformPointCloud(untrans_pts_in, untrans_pts_in, trans_matrix);
}

class TestCurbExtract
{
public:
    TestCurbExtract(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~TestCurbExtract() {}
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg);

private:
    ros::Publisher pc_curb_pub_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<perception::curb::BaseCurbExtract> curbExtract_ptr_;
};

TestCurbExtract::TestCurbExtract(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh)
{
    curbExtract_ptr_.reset(new perception::curb::HDiff());
    curbExtract_ptr_->Init();

    pc_curb_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("h_diff_curb_points", 1);
    static ros::Subscriber lidar_sub = nh_.subscribe(perception::curb::params::topic, 2, &TestCurbExtract::lidarCallback, this);
}

sensor_msgs::PointCloud2 cloud2msg(const pcl::PointCloud<pcl::PointXYZIRT>& cloud, const std_msgs::Header& header) 
{
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header = header;
    return cloud_ROS;
}

void TestCurbExtract::lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloudmsg) 
{
    pcl::PointCloud<pcl::PointXYZIRT> cloud_in;
    pcl::fromROSMsg(*cloudmsg, cloud_in);
    //ptsTrans(cloud_in);
    pcl::PointCloud<pcl::PointXYZIRT> cloud_curb;

    double time_start = ros::Time::now().toSec();
    curbExtract_ptr_->Processing(cloud_in, cloud_curb);
    double time_end = ros::Time::now().toSec();
    std::cout << 3 << '\n';
    std::cout << "HDiff_curb_extract using " << (time_end - time_start) * 1000.0 << " ms\n\n";
    pc_curb_pub_.publish(cloud2msg(cloud_curb, cloudmsg->header));
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_HDiff_features_node");

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh("~");

    TestCurbExtract extract_curb(nh_, private_nh);
    ros::spin();
    return 0;
}
