#include "btc_init_localizer/BTC.h"
#include "btc_init_localizer/btc_database.hpp"

#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>

namespace bil = btc_init_localizer;

class BTCMapBuilder : public rclcpp::Node {
public:
  BTCMapBuilder() : Node("btc_map_builder_node") {
    this->declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    this->declare_parameter<std::string>("odom_topic", "/Odometry");
    this->declare_parameter<std::string>("output_dir", "/tmp/btc_db");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<int>("save_every_n", 3);
    this->declare_parameter<int>("sync_queue_size", 20);
    this->declare_parameter<int>("is_high_fly", 0);

    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("output_dir", output_dir_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("save_every_n", save_every_n_);
    this->get_parameter("sync_queue_size", sync_queue_size_);
    this->get_parameter("is_high_fly", is_high_fly_);

    read_parameters(*this, config_setting_, is_high_fly_);

    boost::filesystem::create_directories(boost::filesystem::path(output_dir_) / "clouds");
    boost::filesystem::create_directories(boost::filesystem::path(output_dir_) / "planes");
    boost::filesystem::create_directories(boost::filesystem::path(output_dir_) / "entries");

    saved_count_ = countExistingEntries();

    cloud_sub_.subscribe(this, cloud_topic_, rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data);
    sync_.reset(new Sync(SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_));
    sync_->registerCallback(std::bind(&BTCMapBuilder::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(), "btc_map_builder ready. cloud_topic=" << cloud_topic_
                    << " odom_topic=" << odom_topic_
                    << " output_dir=" << output_dir_);
  }

private:
  int countExistingEntries() const {
    const std::string index_path = (boost::filesystem::path(output_dir_) / "index.txt").string();
    std::ifstream ifs(index_path);
    if (!ifs.is_open()) {
      return 0;
    }

    int count = 0;
    std::string line;
    while (std::getline(ifs, line)) {
      if (!line.empty()) {
        ++count;
      }
    }
    return count;
  }

  void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                    const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
    ++recv_count_;
    if (save_every_n_ > 1 && (recv_count_ % save_every_n_) != 0) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*cloud_msg, *cloud);
    if (cloud->empty()) {
      return;
    }

    STDescManager desc_manager(config_setting_);
    desc_manager.current_frame_id_ = static_cast<unsigned int>(saved_count_);

    std::vector<STD> stds;
    desc_manager.GenerateSTDescs(cloud, stds, saved_count_);
    if (stds.empty() || desc_manager.plane_cloud_vec_.empty() || !desc_manager.plane_cloud_vec_.back() ||
        desc_manager.plane_cloud_vec_.back()->empty()) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Skip frame due to empty BTC descriptors or empty plane cloud");
      return;
    }

    bil::DatabaseEntry entry;
    entry.index = saved_count_;
    entry.position = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
    entry.orientation = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                           odom_msg->pose.pose.orientation.x,
                                           odom_msg->pose.pose.orientation.y,
                                           odom_msg->pose.pose.orientation.z);
    entry.orientation.normalize();

    if (!map_frame_.empty() && !odom_msg->header.frame_id.empty() && odom_msg->header.frame_id != map_frame_) {
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Odometry frame_id is " << odom_msg->header.frame_id
                               << ", expected " << map_frame_);
    }

    std::ostringstream pcd_name;
    pcd_name << "clouds/" << std::setw(6) << std::setfill('0') << entry.index << ".pcd";
    entry.pcd_path = pcd_name.str();

    std::ostringstream plane_name;
    plane_name << "planes/" << std::setw(6) << std::setfill('0') << entry.index << ".pcd";
    entry.plane_path = plane_name.str();

    entry.stds = stds;
    for (auto& std_item : entry.stds) {
      std_item.frame_number_ = entry.index;
    }

    const std::string abs_pcd = (boost::filesystem::path(output_dir_) / entry.pcd_path).string();
    if (pcl::io::savePCDFileBinaryCompressed(abs_pcd, *cloud) != 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to save pcd: " << abs_pcd);
      return;
    }

    const std::string abs_plane = (boost::filesystem::path(output_dir_) / entry.plane_path).string();
    if (pcl::io::savePCDFileBinaryCompressed(abs_plane, *desc_manager.plane_cloud_vec_.back()) != 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to save plane cloud: " << abs_plane);
      return;
    }

    if (!bil::saveDatabaseEntry(output_dir_, entry)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save BTC database entry");
      return;
    }

    ++saved_count_;
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Saved BTC entry count=" << saved_count_ << " std_count=" << entry.stds.size());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<Sync> sync_;

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string output_dir_;
  std::string map_frame_;

  int save_every_n_ = 3;
  int sync_queue_size_ = 20;
  int is_high_fly_ = 0;
  int recv_count_ = 0;
  int saved_count_ = 0;

  ConfigSetting config_setting_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTCMapBuilder>());
  rclcpp::shutdown();
  return 0;
}
