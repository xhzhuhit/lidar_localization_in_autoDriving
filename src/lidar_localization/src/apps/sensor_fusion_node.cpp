#include <ros/ros.h>
#include "sensor_data/gnss_data.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "sensor_data/imu_data.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "sensor_data/velocity_data.hpp"
#include "subscriber/velocity_subscriber.hpp"

int main(int argc, char** argv) {
    std::cout << "sensor_fusion_node" << std::endl;
    ros::init(argc, argv, "sensor_fusion_node");
    ros::NodeHandle nh;

    // std::string cloud_topic, odom_topic;
    // nh.param<std::string>("cloud_topic", cloud_topic, "/synced_cloud");
    // nh.param<std::string>("odom_topic", odom_topic, "/laser_odom");

    std::deque<lidar_slam::GNSSData> gnss_data_buff;
    std::deque<lidar_slam::VelocityData> velocity_data_buff;
    std::deque<lidar_slam::IMUData> imu_data_buff;

    std::shared_ptr<lidar_slam::GNSSSubscriber> gnss_subscriber_ptr = 
        std::make_shared<lidar_slam::GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000);
    std::shared_ptr<lidar_slam::VelocitySubscriber> vel_subscriber_ptr = 
        std::make_shared<lidar_slam::VelocitySubscriber>(nh, "kitti/oxts/gps/vel", 1000);
    std::shared_ptr<lidar_slam::IMUSubscriber> imu_subscriber_ptr = 
        std::make_shared<lidar_slam::IMUSubscriber>(nh, "/kitti/oxts/imu", 1000);
    
    // do fusion here
    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        gnss_subscriber_ptr->ParseData(gnss_data_buff);
        vel_subscriber_ptr->ParseData(velocity_data_buff);
        imu_subscriber_ptr->ParseData(imu_data_buff);

        // while (velocity_data_buff.size() > 0) {
        //     auto vel_data = velocity_data_buff.front();
        //     std::cout << "velocity: " << vel_data.linear_velocity.x << "," 
        //         << vel_data.linear_velocity.y << ", " << vel_data.linear_velocity.z << std::endl;
        //     velocity_data_buff.pop_front();
        // }
        // while (gnss_data_buff.size() > 0) {
        //     auto gnss_data = gnss_data_buff.front();
        //     std::cout << "gnss_data: lon-lat-height: (" << gnss_data.longitude << ", " << gnss_data.latitude
        //         << "," << gnss_data.altitude << "), e-n-u: (" << gnss_data.local_E << ", "
        //         << gnss_data.local_N << ", " << gnss_data.local_U << "), ori-pose: ("  //<< std::endl;
        //         << gnss_data.origin_longitude << ", " << gnss_data.origin_latitude
        //         << ", " << gnss_data.origin_altitude << std::endl;
        //     gnss_data_buff.pop_front();
        // }
        /*only position. speed, imu */

        rate.sleep();
    }

    return 0;
}