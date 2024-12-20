#pragma once
#include <vector>
#include <unordered_set>
#include <queue>

#include <map>
#include "safety_monitoring_system/types.hpp"
#include <rclcpp/rclcpp.hpp>

class VoxelProcessor
{
public:
  VoxelProcessor(const Parameters &params);
  std::vector<Voxel> create_voxel(const std::vector<Point3D> &points);

private:
  Parameters params_;
};

class Clustering
{
public:
  Clustering(const Parameters &params, bool human_mode = false);
  void process_clusters(const std::vector<Point3D> &processed_points, const std::vector<VoxelCluster> &clusters, rclcpp::Time current_time, double dt);
  std::vector<VoxelCluster> create_voxel_clustering(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels);

  const std::unordered_set<size_t> &get_human_size_cluster_indices() const { return human_size_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_cluster_indices() const { return dynamic_cluster_indices_; }
  const std::unordered_set<size_t> &get_dynamic_human_cluster_indices() const { return dynamic_human_cluster_indices_; }

private:
  Parameters params_;
  bool human_mode_;

  std::unordered_set<size_t> human_size_cluster_indices_;
  std::unordered_set<size_t> dynamic_cluster_indices_;
  std::unordered_set<size_t> dynamic_human_cluster_indices_;

  std::map<int, ClusterTrack> tracks_;

  bool point_in_voxel(const Point3D &point, const Voxel &voxel) const;
  void collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points);
  void calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points, double &size_x, double &size_y, double &size_z) const;

  bool is_human_size(double size_x, double size_y, double size_z) const;
  Point3D calculate_cluster_centroid(const VoxelCluster &cluster);
  void calc_human_clusters_indices(const std::vector<VoxelCluster> &clusters, const std::vector<Point3D> &points);
  std::vector<int> associate_clusters(const std::vector<VoxelCluster> &current_clusters,
                                      std::map<int, ClusterTrack> &tracks,
                                      double max_distance_for_association,
                                      rclcpp::Time current_time,
                                      double dt);
  std::vector<VoxelCluster> identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters, const rclcpp::Time &current_time, double dt);
  std::vector<VoxelCluster> filter_by_speed(const std::vector<VoxelCluster> &current_clusters,
                                            const std::vector<int> &assignments,
                                            std::map<int, ClusterTrack> &tracks,
                                            double dt,
                                            double speed_threshold);
  void remove_missing_tracks();
  void calc_dynamic_cluster_indices(const std::vector<VoxelCluster> &clusters, const std::vector<VoxelCluster> &dynamic_clusters);
  void calc_dynamic_human_cluster_indices(const std::vector<VoxelCluster> &clusters);
  void refine_human_clusters(std::vector<VoxelCluster> &clusters, const Point3D &human_position);
  void filter_dynamic_human_clusters_near_boundaries(const std::vector<VoxelCluster> &clusters);

  std::vector<std::string> get_adjacent_voxels(const std::string &key) const;
};

