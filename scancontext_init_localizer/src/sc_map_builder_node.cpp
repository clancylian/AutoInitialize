#include "scancontext_init_localizer/scan_context.hpp"

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

namespace scl = scancontext_init_localizer;

class SCMapBuilder : public rclcpp::Node {
public:
  SCMapBuilder()
      : Node("sc_map_builder_node") {
    this->declare_parameter<std::string>("cloud_topic", "/velodyne_points");
    this->declare_parameter<std::string>("odom_topic", "/Odometry");
    this->declare_parameter<std::string>("output_dir", "/tmp/scancontext_db");
    this->declare_parameter<int>("save_every_n", 10);
    this->declare_parameter<int>("sync_queue_size", 20);
    this->declare_parameter<std::string>("map_frame", "map");

    this->get_parameter("cloud_topic", cloud_topic_);
    this->get_parameter("odom_topic", odom_topic_);
    this->get_parameter("output_dir", output_dir_);
    this->get_parameter("save_every_n", save_every_n_);
    this->get_parameter("sync_queue_size", sync_queue_size_);
    this->get_parameter("map_frame", map_frame_);

    scl::ScanContext::Params sc_params;
    this->declare_parameter<int>("num_rings", 20);
    this->declare_parameter<int>("num_sectors", 60);
    this->declare_parameter<double>("max_radius", 80.0);
    this->declare_parameter<double>("lidar_height", 2.0);
    this->get_parameter("num_rings", sc_params.num_rings);
    this->get_parameter("num_sectors", sc_params.num_sectors);
    this->get_parameter("max_radius", sc_params.max_radius);
    this->get_parameter("lidar_height", sc_params.lidar_height);
    sc_.reset(new scl::ScanContext(sc_params));

    boost::filesystem::create_directories((boost::filesystem::path(output_dir_) / "clouds"));
    boost::filesystem::create_directories((boost::filesystem::path(output_dir_) / "entries"));
    saved_count_ = countExistingEntries();

    cloud_sub_.subscribe(this, cloud_topic_, rmw_qos_profile_sensor_data);
    odom_sub_.subscribe(this, odom_topic_, rmw_qos_profile_sensor_data);
    sync_.reset(new Sync(SyncPolicy(sync_queue_size_), cloud_sub_, odom_sub_));
    sync_->registerCallback(std::bind(&SCMapBuilder::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO_STREAM(this->get_logger(), "sc_map_builder ready. cloud_topic=" << cloud_topic_
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

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    if (cloud.empty()) {
      return;
    }

    scl::DatabaseEntry entry;
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

    const auto desc = sc_->makeDescriptor(cloud);
    entry.descriptor = desc;
    entry.ring_key = sc_->makeRingKey(desc);

    const std::string abs_pcd = (boost::filesystem::path(output_dir_) / entry.pcd_path).string();
    if (pcl::io::savePCDFileBinaryCompressed(abs_pcd, cloud) != 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to save pcd: " << abs_pcd);
      return;
    }

    if (!scl::saveDatabaseEntry(output_dir_, entry)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to save scan context entry");
      return;
    }

    ++saved_count_;
    RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Saved SC entry count=" << saved_count_);
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
  std::shared_ptr<Sync> sync_;

  std::unique_ptr<scl::ScanContext> sc_;

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string output_dir_;
  std::string map_frame_;
  int save_every_n_ = 10;
  int sync_queue_size_ = 20;

  int recv_count_ = 0;
  int saved_count_ = 0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCMapBuilder>());
  rclcpp::shutdown();
  return 0;
}
