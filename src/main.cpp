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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
        laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("merged_scan", rclcpp::SystemDefaultsQoS());
        RCLCPP_INFO(this->get_logger(), "Hello");
    }
    private:
    void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser1_ = _msg;
        update_point_cloud_rgb();
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
        //         _msg->ranges[100]);
    }
    void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg) {
        laser2_ = _msg;
        // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
        //         _msg->ranges[100]);
    }
    
    void update_point_cloud_2(){
        refresh_params();
        //pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        pcl::PointCloud<pcl::PointXYZ> cloud_;
        std::vector<std::array<float,3>> scan_data;
        int count = 0;
        
        std::array<float,3> push_data;
        if(show1_){
            for (float i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment){
                float temp_x = laser1_->ranges[count] * std::cos(i) + laser1XOff_;
                float temp_y = laser1_->ranges[count] * std::sin(i) + laser1YOff_;
                push_data[0] = temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180);
                push_data[1] = temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180);
                push_data[2] = laser1ZOff_;
                if (i < (laser1AngleMin_ * M_PI / 180)){

                }else if(i > (laser1AngleMax_ * M_PI / 180)){

                }else{
                    scan_data.push_back(push_data);
                }
                count++;
            }
        }
        
        count = 0;
        if(show2_){
            for (float i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment){
                float temp_x = laser2_->ranges[count] * std::cos(i) + laser2XOff_;
                float temp_y = laser2_->ranges[count] * std::sin(i) + laser2YOff_;
                push_data[0] = temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180);
                push_data[1] = temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180);
                push_data[2] = laser2ZOff_;
                if (i < (laser2AngleMin_ * M_PI / 180)){

                }else if(i > (laser2AngleMax_ * M_PI / 180)){

                }else{
                    scan_data.push_back(push_data);
                }
                count++;
            }
        }
        sensor_msgs::msg::PointCloud2 dummy_cloud;
        sensor_msgs::PointCloud2Modifier modifier(dummy_cloud);
        modifier.setPointCloud2Fields(
            3,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(scan_data.size());
        sensor_msgs::PointCloud2Iterator<float> it_x(dummy_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> it_y(dummy_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> it_z(dummy_cloud, "z");
        unsigned long int counter_temp = 0;
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
            *it_x = scan_data[counter_temp][0];
            *it_y = scan_data[counter_temp][1];
            *it_z = scan_data[counter_temp][2];
            counter_temp+=1;
        }
        dummy_cloud.header.frame_id = cloudFrameId_;
        dummy_cloud.header.stamp = now();
        point_cloud_pub_->publish(dummy_cloud);

    }
    void update_point_cloud_rgb(){
        refresh_params();
        pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        //pcl::PointCloud<pcl::PointXYZ> cloud_;
        std::vector<std::array<float,2>> scan_data;
        int count = 0;
        float min_theta = 0;
        float max_theta = 0;
        if(show1_){
            for (float i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment){
                pcl::PointXYZRGB pt;
                pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
                
                //pcl::PointXYZ pt;
                float temp_x = laser1_->ranges[count] * std::cos(i) ;
                float temp_y = laser1_->ranges[count] * std::sin(i) ;
                pt.x = temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180) + laser1XOff_;
                pt.y = temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180) + laser1YOff_;
                pt.z = laser1ZOff_;
                if (i < (laser1AngleMin_ * M_PI / 180)){

                }else if(i > (laser1AngleMax_ * M_PI / 180)){

                }else{
                    //if(!(isnan(pt.x)) && !(isnan(pt.y))){
                        cloud_.points.push_back(pt);
                    float r_ = GET_R(pt.x, pt.y);
                    float theta_ = GET_THETA(pt.x, pt.y);
                    std::array<float,2> res_;
                    res_[1] = r_;
                    res_[0] = theta_;
                    scan_data.push_back(res_);
                    if(theta_ < min_theta){
                        min_theta = theta_;
                    }
                    if(theta_ > max_theta){
                        max_theta = theta_;
                    }
                    //}
                    
                }
                count++;
            }
        }
        
        count = 0;
        if(show2_){
            for (float i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment){
                pcl::PointXYZRGB pt;
                pt = pcl::PointXYZRGB(laser2R_, laser2G_, laser2B_);
                
                //pcl::PointXYZ pt;
                float temp_x = laser2_->ranges[count] * std::cos(i) ;
                float temp_y = laser2_->ranges[count] * std::sin(i) ;
                pt.x = temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180) + laser2XOff_;
                pt.y = temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180)+ laser2YOff_;
                pt.z = laser2ZOff_;
                if (i < (laser2AngleMin_ * M_PI / 180)){

                }else if(i > (laser2AngleMax_ * M_PI / 180)){

                }else{
                    //if(!(isnan(pt.x)) && !(isnan(pt.y))){
                        cloud_.points.push_back(pt);
                    float r_ = GET_R(pt.x, pt.y);
                    float theta_ = GET_THETA(pt.x, pt.y);
                    std::array<float,2> res_;
                    res_[1] = r_;
                    res_[0] = theta_;
                    scan_data.push_back(res_);
                    if(theta_ < min_theta){
                        min_theta = theta_;
                    }
                    if(theta_ > max_theta){
                        max_theta = theta_;
                    }
                    //}
                    
                }
                count++;
            }
        }
        

        auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msg_);
        pc2_msg_->header.frame_id = cloudFrameId_;
        pc2_msg_->header.stamp = now();
        pc2_msg_->is_dense = false;
        point_cloud_pub_->publish(*pc2_msg_);

        
    }

    void update_point_cloud(){
        refresh_params();
        //pcl::PointCloud<pcl::PointXYZRGB> cloud_;
        pcl::PointCloud<pcl::PointXYZ> cloud_;
        std::vector<std::array<float,2>> scan_data;
        int count = 0;
        float min_theta = 0;
        float max_theta = 0;
        if(show1_){
            for (float i = laser1_->angle_min; i <= laser1_->angle_max; i += laser1_->angle_increment){
                //pcl::PointXYZRGB pt;
                //pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
                
                pcl::PointXYZ pt;
                float temp_x = laser1_->ranges[count] * std::cos(i) + laser1XOff_;
                float temp_y = laser1_->ranges[count] * std::sin(i) + laser1YOff_;
                pt.x = temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180);
                pt.y = temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180);
                pt.z = laser1ZOff_;
                if (i < (laser1AngleMin_ * M_PI / 180)){

                }else if(i > (laser1AngleMax_ * M_PI / 180)){

                }else{
                    //if(!(isnan(pt.x)) && !(isnan(pt.y))){
                        cloud_.points.push_back(pt);
                    float r_ = GET_R(pt.x, pt.y);
                    float theta_ = GET_THETA(pt.x, pt.y);
                    std::array<float,2> res_;
                    res_[1] = r_;
                    res_[0] = theta_;
                    scan_data.push_back(res_);
                    if(theta_ < min_theta){
                        min_theta = theta_;
                    }
                    if(theta_ > max_theta){
                        max_theta = theta_;
                    }
                    //}
                    
                }
                count++;
            }
        }
        
        count = 0;
        if(show2_){
            for (float i = laser2_->angle_min; i <= laser2_->angle_max; i += laser2_->angle_increment){
                //pcl::PointXYZRGB pt;
                //pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
                
                pcl::PointXYZ pt;
                float temp_x = laser2_->ranges[count] * std::cos(i) + laser2XOff_;
                float temp_y = laser2_->ranges[count] * std::sin(i) + laser2YOff_;
                pt.x = temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180);
                pt.y = temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180);
                pt.z = laser2ZOff_;
                if (i < (laser2AngleMin_ * M_PI / 180)){

                }else if(i > (laser2AngleMax_ * M_PI / 180)){

                }else{
                    //if(!(isnan(pt.x)) && !(isnan(pt.y))){
                        cloud_.points.push_back(pt);
                    float r_ = GET_R(pt.x, pt.y);
                    float theta_ = GET_THETA(pt.x, pt.y);
                    std::array<float,2> res_;
                    res_[1] = r_;
                    res_[0] = theta_;
                    scan_data.push_back(res_);
                    if(theta_ < min_theta){
                        min_theta = theta_;
                    }
                    if(theta_ > max_theta){
                        max_theta = theta_;
                    }
                    //}
                    
                }
                count++;
            }
        }
        // std::cout << "Smalles angle ";
        // std::cout << min_theta
        //RCLCPP_INFO(this->get_logger(), "Angle range : '%f' - '%f'", min_theta, max_theta);
        
        //std::sort(scan_data.begin(), scan_data.end());

        // for (unsigned long int i = 0; i < scan_data.size(); i++){
        //     RCLCPP_INFO(this->get_logger(), "Angle range : '%f', '%f'", scan_data[i][0], scan_data[i][1]);
        // }

        auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msg_);
        pc2_msg_->header.frame_id = cloudFrameId_;
        pc2_msg_->header.stamp = now();
        pc2_msg_->is_dense = false;
        point_cloud_pub_->publish(*pc2_msg_);

        
    }
    float GET_R(float x, float y){
        return sqrt(x*x + y*y);
    }
    float GET_THETA(float x, float y){
        float temp_res;
        if((x!=0)){
            temp_res = atan(y/x);
        }else{
            
            if(y>=0){
                temp_res = M_PI / 2;
            }else{
                temp_res = -M_PI / 2;
            }
            
        }
        if(temp_res > 0){
            if(y < 0 ){
                temp_res -= M_PI;
            }
        }else if(temp_res <0){
            if(x < 0){
                temp_res += M_PI;
            }
        }
        //RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', a: '%f'", x, y, temp_res);
        
        return temp_res;
    }
    float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle){
        
        return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1)/(angle_2 - angle_1)));
    }
    void initialize_params(){
        
        this->declare_parameter("pointCloudTopic");
        this->declare_parameter("pointCloutFrameId");

        this->declare_parameter("scanTopic1");
        this->declare_parameter("laser1XOff");
        this->declare_parameter("laser1YOff");
        this->declare_parameter("laser1ZOff");
        this->declare_parameter("laser1Alpha");
        this->declare_parameter("laser1AngleMin");
        this->declare_parameter("laser1AngleMax");
        this->declare_parameter("laser1R");
        this->declare_parameter("laser1G");
        this->declare_parameter("laser1B");
        this->declare_parameter("show1");

        this->declare_parameter("scanTopic2");
        this->declare_parameter("laser2XOff");
        this->declare_parameter("laser2YOff");
        this->declare_parameter("laser2ZOff");
        this->declare_parameter("laser2Alpha");
        this->declare_parameter("laser2AngleMin");
        this->declare_parameter("laser2AngleMax");
        this->declare_parameter("laser2R");
        this->declare_parameter("laser2G");
        this->declare_parameter("laser2B");
        this->declare_parameter("show2");

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
        this->get_parameter_or<int>("laser1R",laser1R_, 0);
        this->get_parameter_or<int>("laser1G",laser1G_, 0);
        this->get_parameter_or<int>("laser1B",laser1B_, 0);
        this->get_parameter_or<bool>("show1",show1_, true);
        this->get_parameter_or<std::string>("scanTopic2",topic2_, "lidar_rear_left/scan");
        this->get_parameter_or<float>("laser2XOff",laser2XOff_, 0.0);
        this->get_parameter_or<float>("laser2YOff",laser2YOff_, 0.0);
        this->get_parameter_or<float>("laser2ZOff",laser2ZOff_, 0.0);
        this->get_parameter_or<float>("laser2Alpha",laser2Alpha_, 0.0);
        this->get_parameter_or<float>("laser2AngleMin",laser2AngleMin_,-181.0);
        this->get_parameter_or<float>("laser2AngleMax",laser2AngleMax_, 181.0);
        this->get_parameter_or<int>("laser2R",laser2R_, 0);
        this->get_parameter_or<int>("laser2G",laser2G_, 0);
        this->get_parameter_or<int>("laser2B",laser2B_, 0);
        this->get_parameter_or<bool>("show2",show2_, false);

        
    }
    std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
    bool show1_, show2_;
    float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
    int laser1R_, laser1G_, laser1B_;

    float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
    int laser2R_, laser2G_, laser2B_;

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

