#include "btc_init_localizer/BTC.h"
#include "btc_init_localizer/btc_database.hpp"

#include <boost/filesystem.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace bil = btc_init_localizer;

class BTCNDTInitializer : public rclcpp::Node {
public:
  BTCNDTInitializer() : Node("btc_ndt_initializer_node") {
    this->declare_parameter<std::string>("db_dir", "/tmp/btc_db");
    this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
    this->declare_parameter<double>("service_wait_cloud_sec", 1.0);
    this->declare_parameter<double>("btc_score_threshold", 0.15);
    this->declare_parameter<int>("is_high_fly", 0);

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
    this->get_parameter("btc_score_threshold", btc_score_threshold_);
    this->get_parameter("is_high_fly", is_high_fly_);

    this->get_parameter("ndt_resolution", ndt_resolution_);
    this->get_parameter("ndt_step_size", ndt_step_size_);
    this->get_parameter("ndt_trans_eps", ndt_trans_eps_);
    this->get_parameter("ndt_max_iter", ndt_max_iter_);
    this->get_parameter("ndt_num_threads", ndt_num_threads_);
    this->get_parameter("ndt_neighborhood", ndt_neighborhood_);

    read_parameters(*this, config_setting_, is_high_fly_);
    this->declare_parameter<int>("skip_near_num", -1);
    this->get_parameter("skip_near_num", config_setting_.skip_near_num_);
    config_setting_.icp_threshold_ = static_cast<float>(btc_score_threshold_);

    std::vector<bil::DatabaseEntry> loaded_entries;
    std::string error;
    if (!bil::loadDatabase(db_dir_, &loaded_entries, &error)) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to load BTC database: " << error << " db_dir=" << db_dir_);
      rclcpp::shutdown();
      return;
    }

    desc_manager_.reset(new STDescManager(config_setting_));

    for (auto& entry : loaded_entries) {
      if (entry.stds.empty() || !entry.plane_cloud || entry.plane_cloud->empty()) {
        continue;
      }

      const int local_index = static_cast<int>(entries_.size());
      entry.index = local_index;
      for (auto& std_item : entry.stds) {
        std_item.frame_number_ = local_index;
      }

      desc_manager_->plane_cloud_vec_.push_back(entry.plane_cloud);
      desc_manager_->AddSTDescs(entry.stds);
      entries_.push_back(entry);
    }

    if (entries_.empty()) {
      RCLCPP_FATAL_STREAM(this->get_logger(), "BTC database is empty after filtering. db_dir=" << db_dir_);
      rclcpp::shutdown();
      return;
    }

    initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic_, 1);

    // 使用 BEST_EFFORT QoS 以兼容发布者
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, qos, std::bind(&BTCNDTInitializer::cloudCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO_STREAM(this->get_logger(), "btc_ndt_initializer ready: entries=" << entries_.size());
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    latest_cloud_ = cloud_msg;
    performInitialization();
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

    std::vector<STD> query_stds;
    desc_manager_->GenerateSTDescs(source_cloud, query_stds, static_cast<int>(desc_manager_->current_frame_id_));

    if (query_stds.empty() || desc_manager_->plane_cloud_vec_.empty() ||
        !desc_manager_->plane_cloud_vec_.back() || desc_manager_->plane_cloud_vec_.back()->empty()) {
      if (!desc_manager_->plane_cloud_vec_.empty()) {
        desc_manager_->plane_cloud_vec_.pop_back();
      }
      RCLCPP_WARN(this->get_logger(), "Failed to generate BTC descriptors");
      return false;
    }

    auto query_plane_cloud = desc_manager_->plane_cloud_vec_.back();

    std::pair<int, double> loop_result(-1, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    std::vector<std::pair<STD, STD>> loop_std_pair;
    desc_manager_->SearchLoop(query_stds, loop_result, loop_transform, loop_std_pair, query_plane_cloud);

    desc_manager_->plane_cloud_vec_.pop_back();

    if (loop_result.first < 0 || loop_result.first >= static_cast<int>(entries_.size())) {
      RCLCPP_WARN(this->get_logger(), "BTC matching failed");
      return false;
    }

    if (loop_result.second < btc_score_threshold_) {
      RCLCPP_WARN_STREAM(this->get_logger(), "BTC score below threshold: " << loop_result.second);
      return false;
    }

    const auto& matched = entries_[loop_result.first];

    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    std::string target_path = matched.pcd_path;
    if (!boost::filesystem::path(target_path).is_absolute()) {
      target_path = (boost::filesystem::path(db_dir_) / target_path).string();
    }
    if (pcl::io::loadPCDFile(target_path, *target_cloud) != 0 || target_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load matched target pcd");
      return false;
    }

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    init_guess.block<3, 3>(0, 0) = loop_transform.second.cast<float>();
    init_guess(0, 3) = static_cast<float>(loop_transform.first.x());
    init_guess(1, 3) = static_cast<float>(loop_transform.first.y());
    init_guess(2, 3) = static_cast<float>(loop_transform.first.z());

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

    // 动态获取 world 到 map 的变换
    Eigen::Matrix4f tf_world_to_map = Eigen::Matrix4f::Identity();
    if (!getWorldToMapTransform(tf_world_to_map)) {
      RCLCPP_WARN(this->get_logger(), "Using identity transform from world to map");
    }

    // 将 world 坐标系下的位姿转换到 map 坐标系
    // T_map = T_world_to_map × T_world
    //Eigen::Matrix4f tf_map_from_world = tf_world_to_map * tf_map_current;
    Eigen::Matrix4f tf_map_from_world = tf_map_current;

    RCLCPP_INFO_STREAM(this->get_logger(), "World coordinate: [" << tf_map_current(0, 3)
                    << ", " << tf_map_current(1, 3) << ", " << tf_map_current(2, 3) << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "Map coordinate: [" << tf_map_from_world(0, 3)
                    << ", " << tf_map_from_world(1, 3) << ", " << tf_map_from_world(2, 3) << "]");

    // 使用转换后的 map 坐标系位姿
    Eigen::Quaternionf q(tf_map_from_world.block<3, 3>(0, 0));
    q.normalize();

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = map_frame_;
    // 使用 world->map 转换后的坐标
    pose_msg.pose.pose.position.x = tf_map_from_world(0, 3);
    pose_msg.pose.pose.position.y = tf_map_from_world(1, 3);
    pose_msg.pose.pose.position.z = tf_map_from_world(2, 3);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    std::fill(pose_msg.pose.covariance.begin(), pose_msg.pose.covariance.end(), 0.0);
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.2;

    initialpose_pub_->publish(pose_msg);

    RCLCPP_INFO_STREAM(this->get_logger(), "Init pose published. idx=" << matched.index
                    << " btc_score=" << loop_result.second
                    << " ndt_fitness=" << ndt.getFitnessScore());

    RCLCPP_INFO_STREAM(this->get_logger(), "pose_msg: header.frame_id=" << pose_msg.header.frame_id
                    << " stamp=" << pose_msg.header.stamp.sec << "." << pose_msg.header.stamp.nanosec);
    RCLCPP_INFO_STREAM(this->get_logger(), "  position: x=" << pose_msg.pose.pose.position.x
                    << " y=" << pose_msg.pose.pose.position.y
                    << " z=" << pose_msg.pose.pose.position.z);
    RCLCPP_INFO_STREAM(this->get_logger(), "  orientation: x=" << pose_msg.pose.pose.orientation.x
                    << " y=" << pose_msg.pose.pose.orientation.y
                    << " z=" << pose_msg.pose.pose.orientation.z
                    << " w=" << pose_msg.pose.pose.orientation.w);
    RCLCPP_INFO_STREAM(this->get_logger(), "  covariance[0]=" << pose_msg.pose.covariance[0]
                    << " covariance[7]=" << pose_msg.pose.covariance[7]
                    << " covariance[35]=" << pose_msg.pose.covariance[35]);

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

  bool getWorldToMapTransform(Eigen::Matrix4f& tf_world_to_map) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          "rkbot/map", "rkbot/world",
          tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Could not get transform from rkbot/world to rkbot/map: " << ex.what());
      return false;
    }

    tf_world_to_map = Eigen::Matrix4f::Identity();
    tf_world_to_map(0, 3) = transform_stamped.transform.translation.x;
    tf_world_to_map(1, 3) = transform_stamped.transform.translation.y;
    tf_world_to_map(2, 3) = transform_stamped.transform.translation.z;

    Eigen::Quaternionf q(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z);
    q.normalize();
    tf_world_to_map.block<3, 3>(0, 0) = q.toRotationMatrix();

    RCLCPP_INFO_STREAM(this->get_logger(), "Dynamic transform world->map: trans=[" 
                    << tf_world_to_map(0, 3) << ", " << tf_world_to_map(1, 3) 
                    << ", " << tf_world_to_map(2, 3) << "]");
    return true;
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::unique_ptr<STDescManager> desc_manager_;
  std::vector<bil::DatabaseEntry> entries_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  std::string db_dir_;
  std::string lidar_topic_;
  std::string map_frame_;
  std::string initialpose_topic_;

  double wait_cloud_sec_ = 1.0;
  double btc_score_threshold_ = 0.15;
  int is_high_fly_ = 0;

  double ndt_resolution_ = 1.0;
  double ndt_step_size_ = 0.1;
  double ndt_trans_eps_ = 0.01;
  int ndt_max_iter_ = 40;
  int ndt_num_threads_ = 4;
  int ndt_neighborhood_ = 2;

  ConfigSetting config_setting_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BTCNDTInitializer>());
  rclcpp::shutdown();
  return 0;
}
