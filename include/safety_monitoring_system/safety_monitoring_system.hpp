#ifndef SAFETY_MONITORING_SYSTEM_HPP
#define SAFETY_MONITORING_SYSTEM_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mutex>
#include "safety_monitoring_system/types.hpp"
#include "safety_monitoring_system/clustering.hpp" 
#include <deque>

class SafetyMonitoringSystem : public rclcpp::Node
{
public:
  explicit SafetyMonitoringSystem(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  explicit SafetyMonitoringSystem(const std::string &name_space, const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void load_parameters();
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  visualization_msgs::msg::Marker create_human_marker(const Point3D &centroid, const std_msgs::msg::Header &header);
  visualization_msgs::msg::MarkerArray create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters);

  void publish_markers(Point3D &human_position, const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud);
  Point3D calculate_human_position(const std::vector<VoxelCluster> &clusters);
  std::vector<Point3D> PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg) const;
  sensor_msgs::msg::PointCloud2 vector_to_PC2(const std::vector<Point3D> &points) const;
  std::vector<Point3D> filter_points(const std::vector<Point3D> &points) const;
  bool is_in_restricted_area(const Point3D &position) const;
  visualization_msgs::msg::Marker create_restricted_area_marker(const std_msgs::msg::Header &header);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clustered_voxel_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr safety_area_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;

  std::string frame_id_ = "map";

  Parameters params_;

  std::vector<Point3D> previous_centroids_;

  std::unique_ptr<VoxelProcessor> voxel_processor_;
  std::unique_ptr<Clustering> clustering_;

  rclcpp::Time previous_time_;
  std::mutex centroid_mutex_;
};

#endif // SAFETY_MONITORING_SYSTEM_HPP
