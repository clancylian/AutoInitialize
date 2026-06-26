#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

class TfConversionTest : public rclcpp::Node {
public:
  TfConversionTest() : Node("tf_conversion_test") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/rkbot/lio/odom", 10, 
        std::bind(&TfConversionTest::odomCallback, this, std::placeholders::_1));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "TF conversion test node ready");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // 获取 world 到 map 的变换
    Eigen::Matrix4f tf_world_to_map = Eigen::Matrix4f::Identity();
    if (!getTransform("rkbot/map", "rkbot/world", tf_world_to_map)) {
      RCLCPP_WARN(this->get_logger(), "Cannot get world->map transform, skipping");
      return;
    }

    // 从 odom 消息中提取位姿（在 world 坐标系下）
    Eigen::Matrix4f tf_world_odom = Eigen::Matrix4f::Identity();
    tf_world_odom(0, 3) = odom_msg->pose.pose.position.x;
    tf_world_odom(1, 3) = odom_msg->pose.pose.position.y;
    tf_world_odom(2, 3) = odom_msg->pose.pose.position.z;
    
    Eigen::Quaternionf q(
        odom_msg->pose.pose.orientation.w,
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z);
    tf_world_odom.block<3, 3>(0, 0) = q.toRotationMatrix();

    // 将 world 坐标系下的 odom 位姿转换到 map 坐标系
    // T_map_odom = T_world_to_map × T_world_odom
    Eigen::Matrix4f tf_map_odom = tf_world_to_map * tf_world_odom;

    // 输出结果
    RCLCPP_INFO_STREAM(this->get_logger(), "\n=== TF Conversion Test ===");
    RCLCPP_INFO_STREAM(this->get_logger(), "World->Map Transform:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Translation: [" << tf_world_to_map(0, 3) 
                    << ", " << tf_world_to_map(1, 3) << ", " << tf_world_to_map(2, 3) << "]");

    RCLCPP_INFO_STREAM(this->get_logger(), "\nOdometry in World frame:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Position: [" << odom_msg->pose.pose.position.x
                    << ", " << odom_msg->pose.pose.position.y 
                    << ", " << odom_msg->pose.pose.position.z << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Orientation: [" << odom_msg->pose.pose.orientation.x
                    << ", " << odom_msg->pose.pose.orientation.y 
                    << ", " << odom_msg->pose.pose.orientation.z << ", " << odom_msg->pose.pose.orientation.w << "]");

    RCLCPP_INFO_STREAM(this->get_logger(), "\nOdometry in Map frame:");
    RCLCPP_INFO_STREAM(this->get_logger(), "  Position: [" << tf_map_odom(0, 3)
                    << ", " << tf_map_odom(1, 3) << ", " << tf_map_odom(2, 3) << "]");
    
    Eigen::Quaternionf q_map(tf_map_odom.block<3, 3>(0, 0));
    RCLCPP_INFO_STREAM(this->get_logger(), "  Orientation: [" << q_map.x()
                    << ", " << q_map.y() << ", " << q_map.z() << ", " << q_map.w() << "]");
    RCLCPP_INFO_STREAM(this->get_logger(), "=========================\n");
  }

  bool getTransform(const std::string& target_frame, const std::string& source_frame, 
                    Eigen::Matrix4f& transform) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf_buffer_->lookupTransform(
          target_frame, source_frame,
          tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN_STREAM(this->get_logger(), "TF lookup failed: " << ex.what());
      return false;
    }

    transform = Eigen::Matrix4f::Identity();
    transform(0, 3) = transform_stamped.transform.translation.x;
    transform(1, 3) = transform_stamped.transform.translation.y;
    transform(2, 3) = transform_stamped.transform.translation.z;

    Eigen::Quaternionf q(
        transform_stamped.transform.rotation.w,
        transform_stamped.transform.rotation.x,
        transform_stamped.transform.rotation.y,
        transform_stamped.transform.rotation.z);
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();

    return true;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfConversionTest>());
  rclcpp::shutdown();
  return 0;
}
