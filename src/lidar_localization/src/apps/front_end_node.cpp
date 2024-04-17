#include <deque>
#include <ros/ros.h>
#include "sensor_data/gnss_data.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "sensor_data/cloud_data.hpp"
#include "subscriber/cloud_subscriber.hpp"

int main(int argc, char** argv) {
    std::cout << "front_end_node" << std::endl;
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    std::string cloud_topic, odom_topic;
    nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    std::deque<lidar_slam::GNSSData> gnss_data_buff;
    std::deque<lidar_slam::CloudData> cloud_data_buff;

    std::shared_ptr<lidar_slam::GNSSSubscriber> gnss_subscriber_ptr = 
                    std::make_shared<lidar_slam::GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000);
    std::shared_ptr<lidar_slam::CloudSubscriber> cloud_subscriber_ptr = 
                    std::make_shared<lidar_slam::CloudSubscriber>(nh, "/kitti/velo/pointcloud", 1000);
    //线速度和角度topic:kitti/oxts/gps/vel
    //imu topic: /kitti/oxts/imu

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        gnss_subscriber_ptr->ParseData(gnss_data_buff);
        cloud_subscriber_ptr->ParseData(cloud_data_buff);
    }
    return 0;
}

