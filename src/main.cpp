//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

#include <string>
#include <vector>
#include <array>
#include <iostream>


class scanMerger : public rclcpp::Node
{
    public:
    scanMerger()
    : Node("ros2_laser_scan_merger")
    {
        
        initialize_params();
        refresh_params();
        
        
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
        sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(topic2_, default_qos, std::bind(&scanMerger::scan_callback2 , this, std::placeholders::_1));
        
        point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS());
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(mergedScanTopic_, rclcpp::SystemDefaultsQoS());
        RCLCPP_INFO(this->get_logger(), "Hello");
    }
    private:
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser1_ = _msg;
        update_point_cloud_rgb();
        RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
                _msg->ranges[100]);
    }
    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser2_ = _msg;
        RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
                _msg->ranges[100]);
    }
    
   
   
    void initialize_params(){
        
        this->declare_parameter("pointCloudTopic","base/custom_cloud");
        this->declare_parameter("pointCloutFrameId","laser");

        this->declare_parameter("scanTopic1","lidar_front_right/scan");
        this->declare_parameter("laser1XOff",-0.45);
        this->declare_parameter("laser1YOff",0.24);
        this->declare_parameter("laser1ZOff",0.0);
        this->declare_parameter("laser1Alpha",45.0);
        this->declare_parameter("laser1AngleMin",-181.0);
        this->declare_parameter("laser1AngleMax",181.0);
        this->declare_parameter("laser1R",255);
        this->declare_parameter("laser1G",0);
        this->declare_parameter("laser1B",0);
        this->declare_parameter("show1",true);

        this->declare_parameter("scanTopic2","lidar_rear_left/scan");
        this->declare_parameter("laser2XOff",0.315);
        this->declare_parameter("laser2YOff",-0.24);
        this->declare_parameter("laser2ZOff",0.0);
        this->declare_parameter("laser2Alpha",225.0);
        this->declare_parameter("laser2AngleMin",-181.0);
        this->declare_parameter("laser2AngleMax",181.0);
        this->declare_parameter("laser2R",0);
        this->declare_parameter("laser2G",0);
        this->declare_parameter("laser2B",255);
        this->declare_parameter("show2",true);

        this->declare_parameter("mergedScanTopic","/scan");
    }
    void refresh_params(){
        this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "pointCloud");
        this->get_parameter_or<std::string>("pointCloutFrameId",cloudFrameId_, "laser");
        this->get_parameter_or<std::string>("scanTopic1",topic1_ ,"lidar_front_right/scan");
        this->get_parameter_or<float>("laser1XOff",laser1XOff_, 0.0);
        this->get_parameter_or<float>("laser1YOff",laser1YOff_, 0.0);
        this->get_parameter_or<float>("laser1ZOff",laser1ZOff_, 0.0);
        this->get_parameter_or<float>("laser1Alpha",laser1Alpha_, 0.0);
        this->get_parameter_or<float>("laser1AngleMin",laser1AngleMin_, -181.0);
        this->get_parameter_or<float>("laser1AngleMax",laser1AngleMax_, 181.0);
        this->get_parameter_or<uint8_t>("laser1R",laser1R_, 0);
        this->get_parameter_or<uint8_t>("laser1G",laser1G_, 0);
        this->get_parameter_or<uint8_t>("laser1B",laser1B_, 0);
        this->get_parameter_or<bool>("show1",show1_, true);
        this->get_parameter_or<std::string>("scanTopic2",topic2_, "lidar_rear_left/scan");
        this->get_parameter_or<float>("laser2XOff",laser2XOff_, 0.0);
        this->get_parameter_or<float>("laser2YOff",laser2YOff_, 0.0);
        this->get_parameter_or<float>("laser2ZOff",laser2ZOff_, 0.0);
        this->get_parameter_or<float>("laser2Alpha",laser2Alpha_, 0.0);
        this->get_parameter_or<float>("laser2AngleMin",laser2AngleMin_,-181.0);
        this->get_parameter_or<float>("laser2AngleMax",laser2AngleMax_, 181.0);
        this->get_parameter_or<uint8_t>("laser2R",laser2R_, 0);
        this->get_parameter_or<uint8_t>("laser2G",laser2G_, 0);
        this->get_parameter_or<uint8_t>("laser2B",laser2B_, 0);
        this->get_parameter_or<bool>("show2",show2_, false);
        this->get_parameter_or<std::string>("mergedScanTopic",mergedScanTopic_, "/scan");

        

        
    }
    std::string topic1_, topic2_, cloudTopic_, cloudFrameId_, mergedScanTopic_;
    bool show1_, show2_;
    float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
    uint8_t laser1R_, laser1G_, laser1B_;

    float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
    uint8_t laser2R_, laser2G_, laser2B_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    
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

