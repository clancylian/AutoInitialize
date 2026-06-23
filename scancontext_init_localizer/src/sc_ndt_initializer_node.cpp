#include "scancontext_init_localizer/scan_context.hpp"

#include <boost/filesystem.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace scl = scancontext_init_localizer;

class SCNDTInitializer : public rclcpp::Node {
public:
  SCNDTInitializer() : Node("sc_ndt_initializer_node") {
    this->declare_parameter<std::string>("db_dir", "/tmp/scancontext_db");
    this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
    this->declare_parameter<double>("service_wait_cloud_sec", 1.0);
    this->declare_parameter<int>("num_candidates", 10);
    this->declare_parameter<double>("sc_distance_threshold", 0.25);

    this->declare_parameter<double>("ndt_resolution", 1.0);
    this->declare_parameter<double>("ndt_step_size", 0.1);
    this->declare_parameter<double>("ndt_trans_eps", 0.01);
    this->declare_parameter<int>("ndt_max_iter", 40);
    this->declare_parameter<int>("ndt_num_threads", 4);
    this->declare_parameter<int>("ndt_neighborhood", 2);

    this->get_parameter("db_dir", db_dir_);
    this->get_parameter("lidar_topic", lidar_topic_);
    this->get_parameter("map_frame", map_frame_);
    this->get_parameter("initialpose_topic", initialpose_topic_);
    this->get_parameter("service_wait_cloud_sec", wait_cloud_sec_);
    this->get_parameter("num_candidates", num_candidates_);
    this->get_parameter("sc_distance_threshold", sc_dist_threshold_);

    this->get_parameter("ndt_resolution", ndt_resolution_);
    this->get_parameter("ndt_step_size", ndt_step_size_);
    this->get_parameter("ndt_trans_eps", ndt_trans_eps_);
    this->get_parameter("ndt_max_iter", ndt_max_iter_);
    this->get_parameter("ndt_num_threads", ndt_num_threads_);
    this->get_parameter("ndt_neighborhood", ndt_neighborhood_);

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

    std::string error;
    if (!scl::loadDatabase(db_dir_, sc_params.num_rings, sc_params.num_sectors, &entries_, &error)) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to load SC database: " << error << " db_dir=" << db_dir_);
      rclcpp::shutdown();
      return;
    }

    initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic_, 1);
    
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, 10, std::bind(&SCNDTInitializer::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO_STREAM(this->get_logger(), "sc_ndt_initializer ready: entries=" << entries_.size());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    latest_cloud_ = cloud_msg;
  }

  bool performInitialization() {
    if (!latest_cloud_) {
      RCLCPP_WARN(this->get_logger(), "No cloud received yet");
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*latest_cloud_, *source_cloud);
    if (source_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty lidar cloud");
      return false;
    }

    const auto query_desc = sc_->makeDescriptor(*source_cloud);
    const auto query_key = sc_->makeRingKey(query_desc);

    std::vector<std::pair<float, int>> key_dists;
    key_dists.reserve(entries_.size());
    for (size_t i = 0; i < entries_.size(); ++i) {
      key_dists.emplace_back(sc_->ringKeyDistance(query_key, entries_[i].ring_key), static_cast<int>(i));
    }
    std::sort(key_dists.begin(), key_dists.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    const int eval_num = std::min<int>(num_candidates_, static_cast<int>(key_dists.size()));
    float best_dist = std::numeric_limits<float>::max();
    int best_idx = -1;
    int best_shift = 0;

    for (int i = 0; i < eval_num; ++i) {
      const int idx = key_dists[i].second;
      const auto dist_shift = sc_->distanceWithYaw(query_desc, entries_[idx].descriptor);
      if (dist_shift.first < best_dist) {
        best_dist = dist_shift.first;
        best_idx = idx;
        best_shift = dist_shift.second;
      }
    }

    if (best_idx < 0 || best_dist > sc_dist_threshold_) {
      RCLCPP_WARN_STREAM(this->get_logger(), "SC matching failed or over threshold: " << best_dist);
      return false;
    }

    const auto& matched = entries_[best_idx];
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    std::string target_path = matched.pcd_path;
    if (!boost::filesystem::path(target_path).is_absolute()) {
      target_path = (boost::filesystem::path(db_dir_) / target_path).string();
    }
    if (pcl::io::loadPCDFile(target_path, *target_cloud) != 0 || target_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load matched target pcd");
      return false;
    }

    const double yaw_delta = sc_->sectorAngleRad() * static_cast<double>(best_shift);
    tf2::Quaternion q_guess;
    q_guess.setRPY(0.0, 0.0, yaw_delta);
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf qf(static_cast<float>(q_guess.w()),
                          static_cast<float>(q_guess.x()),
                          static_cast<float>(q_guess.y()),
                          static_cast<float>(q_guess.z()));
    init_guess.block<3, 3>(0, 0) = qf.toRotationMatrix();

    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize(ndt_step_size_);
    ndt.setTransformationEpsilon(ndt_trans_eps_);
    ndt.setMaximumIterations(ndt_max_iter_);

    ndt.setInputTarget(target_cloud);
    ndt.setInputSource(source_cloud);

    pcl::PointCloud<pcl::PointXYZI> aligned;
    ndt.align(aligned, init_guess);

    if (!ndt.hasConverged()) {
      RCLCPP_WARN(this->get_logger(), "NDT did not converge");
      return false;
    }

    const Eigen::Matrix4f tf_historical_current = ndt.getFinalTransformation();
    const Eigen::Matrix4f tf_map_historical = composePoseMatrix(matched.position, matched.orientation);
    const Eigen::Matrix4f tf_map_current = tf_map_historical * tf_historical_current;

    Eigen::Quaternionf q(tf_map_current.block<3, 3>(0, 0));
    q.normalize();

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose.position.x = tf_map_current(0, 3);
    pose_msg.pose.pose.position.y = tf_map_current(1, 3);
    pose_msg.pose.pose.position.z = tf_map_current(2, 3);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.2;

    initialpose_pub_->publish(pose_msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Init pose published. idx=" << matched.index << " sc_dist=" << best_dist
                    << " ndt_fitness=" << ndt.getFitnessScore());
    return true;
  }

private:
  static Eigen::Matrix4f composePoseMatrix(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf qf(static_cast<float>(q.w()),
                          static_cast<float>(q.x()),
                          static_cast<float>(q.y()),
                          static_cast<float>(q.z()));
    qf.normalize();
    m.block<3, 3>(0, 0) = qf.toRotationMatrix();
    m(0, 3) = static_cast<float>(t.x());
    m(1, 3) = static_cast<float>(t.y());
    m(2, 3) = static_cast<float>(t.z());
    return m;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  std::unique_ptr<scl::ScanContext> sc_;
  std::vector<scl::DatabaseEntry> entries_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::string db_dir_;
  std::string lidar_topic_;
  std::string map_frame_;
  std::string initialpose_topic_;
  double wait_cloud_sec_ = 1.0;
  int num_candidates_ = 10;
  double sc_dist_threshold_ = 0.25;

  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_trans_eps_ = 0.01;
  int ndt_max_iter_ = 40;
  int ndt_num_threads_ = 4;
  int ndt_neighborhood_ = 2;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SCNDTInitializer>());
  rclcpp::shutdown();
  return 0;
}
