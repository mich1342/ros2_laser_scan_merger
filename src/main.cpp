#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

#include <string>

class scanMerger : public rclcpp::Node
{
    public:
    scanMerger()
    : Node("ros2_laser_scan_merger")
    {
        this->declare_parameter<std::string>("scanTopic1", "A2/scan");
        this->declare_parameter<std::string>("scanTopic2", "S1/scan");


        this->get_parameter("scanTopic1", topic1_);
        this->get_parameter("scanTopic2", topic2_);
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic2_, default_qos, std::bind(&scanMerger::scan_callback2 , this, std::placeholders::_1));
        
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_merged", rclcpp::SystemDefaultsQoS());
        RCLCPP_INFO(this->get_logger(), "Hello");
    }
    private:
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser1_ = _msg;
        update_point_cloud();
        RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
                _msg->ranges[100]);
    }
    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser2_ = _msg;
        RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
                _msg->ranges[100]);
    }
    void update_point_cloud(){
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        int count = 0;
        for (double i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment){
            pcl::PointXYZRGB pt;
            pt = pcl::PointXYZRGB(255, 0, 0);
            pt.x = laser1_->ranges[count] * std::sin(i);
            pt.y = laser1_->ranges[count] * std::cos(i);
            pt.z = 0.5;
            cloud_.points.push_back(pt);
            count++;
        }
        count = 0;
        for (double i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment){
            pcl::PointXYZRGB pt;
            pt = pcl::PointXYZRGB(0, 255, 0);
            pt.x = laser2_->ranges[count] * std::sin(i);
            pt.y = laser2_->ranges[count] * std::cos(i);
            pt.z = 0.5;
            cloud_.points.push_back(pt);
            count++;
        }
        auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msg_);
        pc2_msg_->header.frame_id = "map";
        pc2_msg_->header.stamp = now();
        point_cloud_pub_->publish(*pc2_msg_);

    }
    std::string topic1_, topic2_;
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

    
    //sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr laser1_;
    sensor_msgs::msg::LaserScan::SharedPtr laser2_;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<scanMerger>());
    rclcpp::shutdown();
    return 0;
}