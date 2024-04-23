#include <deque>
#include <ros/ros.h>
#include "sensor_data/gnss_data.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "sensor_data/cloud_data.hpp"
#include "subscriber/cloud_subscriber.hpp"

#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>

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

    // declare pcl.ndt method
    // method should be in another file: front_end_matching.cpp
    
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.05);
    ndt.setStepSize(0.2);
    ndt.setResolution(2.0);
    ndt.setMaximumIterations(10);

    bool cloud_inited = false;
    lidar_slam::CloudData former_cloud;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        gnss_subscriber_ptr->ParseData(gnss_data_buff);
        cloud_subscriber_ptr->ParseData(cloud_data_buff);
        if (cloud_data_buff.size() > 0) {
            std::cout << "cloud_data_buff_size: " << cloud_data_buff.size() << std::endl;
            if (!cloud_inited) {
                former_cloud = cloud_data_buff[0];
                cloud_inited = true;
                cloud_data_buff.pop_front();
            } else {
                for (int i = 0; i < cloud_data_buff.size(); ++i) {
                    std::cout << "do ndt calc: " << i << std::endl;
                    ndt.setInputTarget((former_cloud.cloud_ptr));
                    ndt.setInputSource((cloud_data_buff[i].cloud_ptr));
                    lidar_slam::CloudData::CLOUD_PTR output_cloud(new lidar_slam::CloudData::CLOUD);

                    Eigen::Matrix4f predict_pose = Eigen::Matrix4f::Identity();
                    //predict_pose.block<3,3>(0,0) = Eigen::

                    ndt.align(*output_cloud, predict_pose);
                    Eigen::Matrix4f result_pose = ndt.getFinalTransformation();
                    
                    //prepare next<
                    former_cloud = cloud_data_buff[i];
                    std::cout << "result-post:  " << result_pose << std::endl;
                }
                cloud_data_buff.clear();
            }
        }

        //在这里调用前端匹配，然后把pose发出去，和点云一起在rviz渲染
        //front_end_matching.match(gnss_data_buff, cloud_data_buff)
    }
    return 0;
}

