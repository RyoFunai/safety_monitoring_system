#include "safety_monitoring_system/safety_monitoring_system.hpp"
#include <memory>
#include <algorithm>
#include <cmath>

SafetyMonitoringSystem::SafetyMonitoringSystem(const rclcpp::NodeOptions &options)
    : SafetyMonitoringSystem("", options)
{
}

SafetyMonitoringSystem::SafetyMonitoringSystem(const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node("safety_monitoring_system", name_space, options)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/livox/lidar", 10, std::bind(&SafetyMonitoringSystem::pointcloud_callback, this, std::placeholders::_1));

  safety_area_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("safety_area", 10);
  filtered_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 10);
  clustered_voxel_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("clustered_voxel", 10);

  this->declare_parameter("min_x", 0.0);
  this->declare_parameter("max_x", 4.0);
  this->declare_parameter("min_y", -3.0);
  this->declare_parameter("max_y", 3.0);
  this->declare_parameter("min_z", 0.1);
  this->declare_parameter("max_z", 2.5);
  this->declare_parameter("voxel_size_x", 0.1);
  this->declare_parameter("voxel_size_y", 0.1);
  this->declare_parameter("voxel_size_z", 0.1);
  this->declare_parameter("voxel_search_range", 1);
  this->declare_parameter("human_height", 2.0);  // 人の高さ目安
  this->declare_parameter("human_width", 0.8);   // 人の幅目安
  this->declare_parameter("human_depth", 0.8);   // 人の奥行き目安
  this->declare_parameter("human_vel_min", 0.5); // 動的とみなす速度閾値
  this->declare_parameter("max_distance_for_association", 1.0);
  this->declare_parameter("missing_count_threshold", 50);
  this->declare_parameter("restricted_min_x", 1.0);
  this->declare_parameter("restricted_max_x", 3.0);
  this->declare_parameter("restricted_min_y", -1.0);
  this->declare_parameter("restricted_max_y", 1.0);
  this->declare_parameter("restricted_min_z", 0.0);
  this->declare_parameter("restricted_max_z", 2.0);

  load_parameters();

  voxel_processor_ = std::make_unique<VoxelProcessor>(params_);
  clustering_ = std::make_unique<Clustering>(params_, true);
}

void SafetyMonitoringSystem::load_parameters()
{
  params_.min_x = this->get_parameter("min_x").as_double();
  params_.max_x = this->get_parameter("max_x").as_double();
  params_.min_y = this->get_parameter("min_y").as_double();
  params_.max_y = this->get_parameter("max_y").as_double();
  params_.min_z = this->get_parameter("min_z").as_double();
  params_.max_z = this->get_parameter("max_z").as_double();
  params_.voxel_size_x = this->get_parameter("voxel_size_x").as_double();
  params_.voxel_size_y = this->get_parameter("voxel_size_y").as_double();
  params_.voxel_size_z = this->get_parameter("voxel_size_z").as_double();
  params_.voxel_search_range = this->get_parameter("voxel_search_range").as_int();
  params_.human_height = this->get_parameter("human_height").as_double();
  params_.human_width = this->get_parameter("human_width").as_double();
  params_.human_depth = this->get_parameter("human_depth").as_double();
  params_.human_vel_min = this->get_parameter("human_vel_min").as_double();
  params_.max_distance_for_association = this->get_parameter("max_distance_for_association").as_double();
  params_.missing_count_threshold = this->get_parameter("missing_count_threshold").as_int();
  params_.restricted_min_x_ = this->get_parameter("restricted_min_x").as_double();
  params_.restricted_max_x_ = this->get_parameter("restricted_max_x").as_double();
  params_.restricted_min_y_ = this->get_parameter("restricted_min_y").as_double();
  params_.restricted_max_y_ = this->get_parameter("restricted_max_y").as_double();
  params_.restricted_min_z_ = this->get_parameter("restricted_min_z").as_double();
  params_.restricted_max_z_ = this->get_parameter("restricted_max_z").as_double();
}

void SafetyMonitoringSystem::pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  rclcpp::Time current_time = this->now();
  double dt = (previous_time_.nanoseconds() == 0) ? 0.0 : (current_time - previous_time_).seconds();
  previous_time_ = current_time;

  std::vector<Point3D> processed_points = PC2_to_vector(*msg);
  processed_points = filter_points(processed_points);
  // ボクセル化 & クラスタリング
  std::vector<Voxel> voxels = voxel_processor_->create_voxel(processed_points);
  std::vector<VoxelCluster> clusters = clustering_->create_voxel_clustering(processed_points, voxels);
  clustering_->process_clusters(processed_points, clusters, current_time, dt);

  Point3D human_position = calculate_human_position(clusters);

  // 残り点群をpublish
  sensor_msgs::msg::PointCloud2 remaining_cloud = vector_to_PC2(processed_points);
  filtered_cloud_publisher_->publish(remaining_cloud);

  // マーカーをpublish
  publish_markers(human_position, clusters, remaining_cloud);

  if (is_in_restricted_area(human_position))
  {
    std::cout << "侵入しています" << std::endl;
  }
}

void SafetyMonitoringSystem::publish_markers(Point3D &human_position, const std::vector<VoxelCluster> &clusters, const sensor_msgs::msg::PointCloud2 &remaining_cloud)
{
  visualization_msgs::msg::Marker restricted_marker = create_restricted_area_marker(remaining_cloud.header);
  safety_area_publisher_->publish(restricted_marker);
  visualization_msgs::msg::MarkerArray voxel_marker_array = create_voxel_cluster_markers(clusters);
  clustered_voxel_publisher_->publish(voxel_marker_array);

  if (human_position.x == 0.0 && human_position.y == 0.0 && human_position.z == 0.0)
  {
    return;
  }

  visualization_msgs::msg::Marker marker = create_human_marker(human_position, remaining_cloud.header);
  safety_area_publisher_->publish(marker);
}

Point3D SafetyMonitoringSystem::calculate_human_position(const std::vector<VoxelCluster> &clusters)
{
  // dynamicでかつhumanサイズのクラスタ
  const auto &dynamic_human_indices = clustering_->get_dynamic_human_cluster_indices();
  std::vector<Point3D> candidate_points;

  for (const auto &index : dynamic_human_indices)
  {
    const VoxelCluster &cluster = clusters[index];
    candidate_points.insert(candidate_points.end(), cluster.points.begin(), cluster.points.end());
  }

  if (candidate_points.size() < 2)
  {
    return Point3D{0.0, 0.0, 0.0};
  }

  std::sort(candidate_points.begin(), candidate_points.end(),
            [](const Point3D &a, const Point3D &b)
            { return a.x < b.x; });

  const Point3D &point1 = candidate_points[0];
  const Point3D &point2 = candidate_points[1];

  return Point3D{(point1.x + point2.x) / 2.0, (point1.y + point2.y) / 2.0, (point1.z + point2.z) / 2.0};
}

visualization_msgs::msg::MarkerArray SafetyMonitoringSystem::create_voxel_cluster_markers(const std::vector<VoxelCluster> &clusters)
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto &human_indices = clustering_->get_human_size_cluster_indices();
  const auto &dynamic_indices = clustering_->get_dynamic_cluster_indices();
  const auto &dynamic_human_indices = clustering_->get_dynamic_human_cluster_indices();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    bool is_human_cluster = (human_indices.find(i) != human_indices.end());
    bool is_dynamic = (dynamic_indices.find(i) != dynamic_indices.end());
    bool is_dynamic_human = (dynamic_human_indices.find(i) != dynamic_human_indices.end());
    float r, g, b;
    if (is_dynamic_human)
    {
      r = 0.0f;
      g = 1.0f;
      b = 0.0f; // 緑: 動的な人
    }
    else if (is_human_cluster)
    {
      r = 1.0f;
      g = 0.0f;
      b = 0.0f; // 赤: 静的な人
    }
    else
    {
      r = 0.0f;
      g = 0.0f;
      b = 1.0f; // 青: その他
    }

    for (size_t v = 0; v < clusters[i].voxels.size(); ++v)
    {
      const auto &voxel = clusters[i].voxels[v];
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.ns = "voxel_cluster_markers";
      marker.id = static_cast<int>(i * 10000 + v);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      double offset_x = params_.voxel_size_x / 2.0;
      double offset_y = params_.voxel_size_y / 2.0;
      double offset_z = params_.voxel_size_z / 2.0;

      marker.pose.position.x = params_.min_x + (voxel.x * params_.voxel_size_x) + offset_x;
      marker.pose.position.y = params_.min_y + (voxel.y * params_.voxel_size_y) + offset_y;
      marker.pose.position.z = params_.min_z + (voxel.z * params_.voxel_size_z) + offset_z;
      marker.pose.orientation.w = 1.0;

      marker.scale.x = params_.voxel_size_x;
      marker.scale.y = params_.voxel_size_y;
      marker.scale.z = params_.voxel_size_z;

      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = 0.3;

      marker.lifetime = rclcpp::Duration(0, 0);

      marker_array.markers.push_back(marker);
    }
  }

  return marker_array;
}

visualization_msgs::msg::Marker SafetyMonitoringSystem::create_human_marker(const Point3D &centroid, const std_msgs::msg::Header &header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = centroid.x;
  marker.pose.position.y = centroid.y;
  marker.pose.position.z = centroid.z;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2; // 人を代表するマーカーサイズは任意
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration(0, 1e8);
  return marker;
}

std::vector<Point3D> SafetyMonitoringSystem::PC2_to_vector(const sensor_msgs::msg::PointCloud2 &cloud_msg) const
{
  std::vector<Point3D> points;
  size_t num_points = cloud_msg.width * cloud_msg.height;
  points.reserve(num_points);

  size_t point_step = cloud_msg.point_step;
  size_t x_offset = cloud_msg.fields[0].offset;
  size_t y_offset = cloud_msg.fields[1].offset;
  size_t z_offset = cloud_msg.fields[2].offset;

  for (size_t i = 0; i < num_points; ++i)
  {
    size_t data_index = i * point_step;
    Point3D point;
    memcpy(&point.x, &cloud_msg.data[data_index + x_offset], sizeof(float));
    memcpy(&point.y, &cloud_msg.data[data_index + y_offset], sizeof(float));
    memcpy(&point.z, &cloud_msg.data[data_index + z_offset], sizeof(float));
    points.push_back(point);
  }

  return points;
}

sensor_msgs::msg::PointCloud2 SafetyMonitoringSystem::vector_to_PC2(const std::vector<Point3D> &points) const
{
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header.frame_id = "map";
  cloud_msg.height = 1;
  cloud_msg.width = points.size();
  cloud_msg.fields.resize(3);
  cloud_msg.fields[0].name = "x";
  cloud_msg.fields[1].name = "y";
  cloud_msg.fields[2].name = "z";

  for (int i = 0; i < 3; ++i)
  {
    cloud_msg.fields[i].offset = i * sizeof(float);
    cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
    cloud_msg.fields[i].count = 1;
  }

  cloud_msg.point_step = 3 * sizeof(float);
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = true;

  for (size_t i = 0; i < points.size(); ++i)
  {
    size_t data_index = i * cloud_msg.point_step;
    memcpy(&cloud_msg.data[data_index], &points[i].x, sizeof(float));
    memcpy(&cloud_msg.data[data_index + sizeof(float)], &points[i].y, sizeof(float));
    memcpy(&cloud_msg.data[data_index + 2 * sizeof(float)], &points[i].z, sizeof(float));
  }

  return cloud_msg;
}

std::vector<Point3D> SafetyMonitoringSystem::filter_points(const std::vector<Point3D> &points) const
{
  std::vector<Point3D> filtered;
  filtered.reserve(points.size());

  for (const auto &point : points)
  {
    if (point.x >= params_.min_x && point.x <= params_.max_x &&
        point.y >= params_.min_y && point.y <= params_.max_y &&
        point.z >= params_.min_z && point.z <= params_.max_z)
    {
      filtered.push_back(point);
    }
  }

  return filtered;
}

bool SafetyMonitoringSystem::is_in_restricted_area(const Point3D &position) const
{
  return (position.x >= params_.restricted_min_x_ && position.x <= params_.restricted_max_x_ &&
          position.y >= params_.restricted_min_y_ && position.y <= params_.restricted_max_y_ &&
          position.z >= params_.restricted_min_z_ && position.z <= params_.restricted_max_z_);
}

visualization_msgs::msg::Marker SafetyMonitoringSystem::create_restricted_area_marker(const std_msgs::msg::Header &header)
{
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.id = 99999;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  double center_x = (params_.restricted_min_x_ + params_.restricted_max_x_) / 2.0;
  double center_y = (params_.restricted_min_y_ + params_.restricted_max_y_) / 2.0;
  double center_z = (params_.restricted_min_z_ + params_.restricted_max_z_) / 2.0;
  double size_x = params_.restricted_max_x_ - params_.restricted_min_x_;
  double size_y = params_.restricted_max_y_ - params_.restricted_min_y_;
  double size_z = params_.restricted_max_z_ - params_.restricted_min_z_;
  marker.pose.position.x = center_x;
  marker.pose.position.y = center_y;
  marker.pose.position.z = center_z;
  marker.scale.x = size_x;
  marker.scale.y = size_y;
  marker.scale.z = size_z;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 0.4;
  marker.lifetime = rclcpp::Duration(0, 0);
  return marker;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyMonitoringSystem>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
