#include "subscriber/cloud_subscriber.hpp"
#include "glog/logging.h"

namespace lidar_slam {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
    approximate_voxel_grid_.setLeafSize(0.8, 0.8, 0.8);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    buff_mutex_.lock();
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));
    //std::cout << "cloud time: " << cloud_data.time << std::endl;
    //do filter here
    lidar_slam::CloudData::CLOUD_PTR filtered_cloud(new lidar_slam::CloudData::CLOUD);
    std::cout << "before filter cloud size: " << cloud_data.cloud_ptr->size() << std::endl;
    approximate_voxel_grid_.setInputCloud(cloud_data.cloud_ptr);
    approximate_voxel_grid_.filter(*filtered_cloud);
    cloud_data.cloud_ptr = filtered_cloud;
    std::cout << "after filter cloud size: " << filtered_cloud->size() << ", " 
        << cloud_data.cloud_ptr->size() << std::endl;

    new_cloud_data_.push_back(cloud_data);
    buff_mutex_.unlock();
}

void CloudSubscriber::ParseData(std::deque<CloudData>& cloud_data_buff) {
    buff_mutex_.lock();
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
    buff_mutex_.unlock();
}
} // namespace data_input
