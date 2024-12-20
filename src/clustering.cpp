#include "safety_monitoring_system/clustering.hpp"
#include <algorithm>
#include <limits>
#include <cmath>
#include <sstream>

VoxelProcessor::VoxelProcessor(const Parameters &params) : params_(params) {}

std::vector<Voxel> VoxelProcessor::create_voxel(const std::vector<Point3D> &points)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto &point : points)
  {
    int vx = static_cast<int>((point.x - params_.min_x) / params_.voxel_size_x);
    int vy = static_cast<int>((point.y - params_.min_y) / params_.voxel_size_y);
    int vz = static_cast<int>((point.z - params_.min_z) / params_.voxel_size_z);

    std::stringstream ss;
    ss << vx << "," << vy << "," << vz;
    std::string key = ss.str();

    if (occupied_voxels.find(key) == occupied_voxels.end())
    {
      occupied_voxels[key] = Voxel(vx, vy, vz);
    }
    else
    {
      occupied_voxels[key].increment();
    }
  }

  std::vector<Voxel> result;
  result.reserve(occupied_voxels.size());
  for (const auto &pair : occupied_voxels)
  {
    result.push_back(pair.second);
  }

  return result;
}

Clustering::Clustering(const Parameters &params, bool human_mode)
    : params_(params), human_mode_(human_mode)
{
}

std::vector<VoxelCluster> Clustering::create_voxel_clustering(const std::vector<Point3D> &points, const std::vector<Voxel> &voxels)
{
  std::unordered_map<std::string, Voxel> occupied_voxels;
  for (const auto &voxel : voxels)
  {
    std::stringstream ss;
    ss << voxel.x << "," << voxel.y << "," << voxel.z;
    std::string key = ss.str();
    occupied_voxels[key] = voxel;
  }

  std::vector<VoxelCluster> clusters;
  std::unordered_map<std::string, bool> visited;
  std::queue<std::string> q;

  for (const auto &pair : occupied_voxels)
  {
    const std::string &key = pair.first;
    if (visited.find(key) != visited.end())
      continue;

    VoxelCluster cluster;
    q.push(key);
    visited[key] = true;

    while (!q.empty())
    {
      std::string current_key = q.front();
      q.pop();
      cluster.voxels.push_back(occupied_voxels[current_key]);

      auto neighbors = get_adjacent_voxels(current_key);
      for (const auto &neighbor_key : neighbors)
      {
        if (occupied_voxels.find(neighbor_key) != occupied_voxels.end() && visited.find(neighbor_key) == visited.end())
        {
          q.push(neighbor_key);
          visited[neighbor_key] = true;
        }
      }
    }

    collect_cluster_points(cluster, points);
    clusters.push_back(cluster);
  }

  return clusters;
}

void Clustering::process_clusters(const std::vector<Point3D> &processed_points, const std::vector<VoxelCluster> &clusters, rclcpp::Time current_time, double dt)
{
  // 人サイズクラスタ抽出
  calc_human_clusters_indices(clusters, processed_points);

  // 動的クラスタ抽出
  std::vector<VoxelCluster> dynamic_clusters = identify_dynamic_clusters(clusters, current_time, dt);
  calc_dynamic_cluster_indices(clusters, dynamic_clusters);

  // 人サイズかつ動的なクラスタ抽出
  calc_dynamic_human_cluster_indices(clusters);

  // 境界付近の動的人クラスタ除外
  filter_dynamic_human_clusters_near_boundaries(clusters);
}

bool Clustering::point_in_voxel(const Point3D &point, const Voxel &voxel) const
{
  return point.x >= params_.min_x + voxel.x * params_.voxel_size_x &&
         point.x < params_.min_x + (voxel.x + 1) * params_.voxel_size_x &&
         point.y >= params_.min_y + voxel.y * params_.voxel_size_y &&
         point.y < params_.min_y + (voxel.y + 1) * params_.voxel_size_y &&
         point.z >= params_.min_z + voxel.z * params_.voxel_size_z &&
         point.z < params_.min_z + (voxel.z + 1) * params_.voxel_size_z;
}

void Clustering::collect_cluster_points(VoxelCluster &cluster, const std::vector<Point3D> &points)
{
  for (const auto &voxel : cluster.voxels)
  {
    for (const auto &point : points)
    {
      if (point_in_voxel(point, voxel))
      {
        cluster.points.push_back(point);
      }
    }
  }
}

void Clustering::calculate_cluster_size(const VoxelCluster &cluster, const std::vector<Point3D> &points,
                                        double &size_x, double &size_y, double &size_z) const
{
  double min_x = std::numeric_limits<double>::max(), max_x = std::numeric_limits<double>::lowest();
  double min_y = std::numeric_limits<double>::max(), max_y = std::numeric_limits<double>::lowest();
  double min_z = std::numeric_limits<double>::max(), max_z = std::numeric_limits<double>::lowest();

  for (const auto &p : cluster.points)
  {
    min_x = std::min(min_x, (double)p.x);
    max_x = std::max(max_x, (double)p.x);
    min_y = std::min(min_y, (double)p.y);
    max_y = std::max(max_y, (double)p.y);
    min_z = std::min(min_z, (double)p.z);
    max_z = std::max(max_z, (double)p.z);
  }

  if (cluster.points.empty())
  {
    size_x = size_y = size_z = 0.0;
    return;
  }

  size_x = max_x - min_x;
  size_y = max_y - min_y;
  size_z = max_z - min_z;
}

bool Clustering::is_human_size(double size_x, double size_y, double size_z) const
{
  // 縦方向(z)がhuman_height以内、横幅(x,y)がhuman_width, human_depth以内とする例
  // パラメータで定義済み: human_height, human_width, human_depth
  return (size_z <= params_.human_height &&
          size_x <= params_.human_width &&
          size_y <= params_.human_depth);
}

Point3D Clustering::calculate_cluster_centroid(const VoxelCluster &cluster)
{
  Point3D centroid{0.0f, 0.0f, 0.0f};
  if (cluster.points.empty())
  {
    return centroid;
  }

  double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
  for (const auto &p : cluster.points)
  {
    sum_x += p.x;
    sum_y += p.y;
    sum_z += p.z;
  }
  double n = static_cast<double>(cluster.points.size());
  centroid.x = (float)(sum_x / n);
  centroid.y = (float)(sum_y / n);
  centroid.z = (float)(sum_z / n);
  return centroid;
}

void Clustering::calc_human_clusters_indices(const std::vector<VoxelCluster> &clusters, const std::vector<Point3D> &points)
{
  human_size_cluster_indices_.clear();

  for (size_t i = 0; i < clusters.size(); ++i)
  {
    double size_x, size_y, size_z;
    calculate_cluster_size(clusters[i], points, size_x, size_y, size_z);
    if (is_human_size(size_x, size_y, size_z))
    {
      human_size_cluster_indices_.insert(i);
    }
  }
}

std::vector<std::string> Clustering::get_adjacent_voxels(const std::string &key) const
{
  int cx, cy, cz;
  sscanf(key.c_str(), "%d,%d,%d", &cx, &cy, &cz);
  std::vector<std::string> neighbors;
  for (int dx = -params_.voxel_search_range; dx <= params_.voxel_search_range; ++dx)
  {
    for (int dy = -params_.voxel_search_range; dy <= params_.voxel_search_range; ++dy)
    {
      for (int dz = -params_.voxel_search_range; dz <= params_.voxel_search_range; ++dz)
      {
        if (dx == 0 && dy == 0 && dz == 0)
          continue;
        std::stringstream ss;
        ss << (cx + dx) << "," << (cy + dy) << "," << (cz + dz);
        neighbors.emplace_back(ss.str());
      }
    }
  }
  return neighbors;
}

std::vector<VoxelCluster> Clustering::identify_dynamic_clusters(const std::vector<VoxelCluster> &clusters, const rclcpp::Time &current_time, double dt)
{
  std::vector<int> assignments = associate_clusters(clusters, tracks_, params_.max_distance_for_association, current_time, dt);
  remove_missing_tracks();
  std::vector<VoxelCluster> dynamic_clusters = filter_by_speed(clusters, assignments, tracks_, dt, params_.human_vel_min);
  return dynamic_clusters;
}

void Clustering::remove_missing_tracks()
{
  for (auto it = tracks_.begin(); it != tracks_.end();)
  {
    if (it->second.missing_count > params_.missing_count_threshold)
      it = tracks_.erase(it);
    else
      ++it;
  }
}

std::vector<int> Clustering::associate_clusters(const std::vector<VoxelCluster> &current_clusters,
                                                std::map<int, ClusterTrack> &tracks,
                                                double max_distance_for_association,
                                                rclcpp::Time current_time,
                                                double dt)
{
  std::vector<int> assignments(current_clusters.size(), -1);
  std::vector<bool> track_used(tracks.size(), false);

  std::vector<std::map<int, ClusterTrack>::iterator> track_list;
  track_list.reserve(tracks.size());
  for (auto it = tracks.begin(); it != tracks.end(); ++it)
  {
    track_list.push_back(it);
  }

  for (size_t i = 0; i < current_clusters.size(); i++)
  {
    Point3D c = calculate_cluster_centroid(current_clusters[i]);
    double best_dist = std::numeric_limits<double>::max();
    int best_idx = -1;
    for (size_t j = 0; j < track_list.size(); j++)
    {
      if (track_used[j])
        continue;
      Point3D t = track_list[j]->second.last_centroid;
      double dx = c.x - t.x;
      double dy = c.y - t.y;
      double dz = c.z - t.z;
      double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (dist < best_dist && dist < max_distance_for_association)
      {
        best_dist = dist;
        best_idx = (int)j;
      }
    }
    if (best_idx >= 0)
    {
      assignments[i] = track_list[best_idx]->first;
      track_used[best_idx] = true;
      track_list[best_idx]->second.last_update_time = current_time;
      track_list[best_idx]->second.missing_count = 0;
    }
  }

  // 新規トラック追加
  for (size_t i = 0; i < assignments.size(); i++)
  {
    if (assignments[i] < 0)
    {
      int new_id = tracks.empty() ? 0 : tracks.rbegin()->first + 1;
      Point3D c = calculate_cluster_centroid(current_clusters[i]);
      ClusterTrack new_track{new_id, c, current_time, 0};
      tracks[new_id] = new_track;
      assignments[i] = new_id;
    }
  }

  // 未割当トラックmissing_count++
  for (size_t j = 0; j < track_list.size(); j++)
  {
    if (!track_used[j])
    {
      track_list[j]->second.missing_count += 1;
    }
  }

  return assignments;
}

std::vector<VoxelCluster> Clustering::filter_by_speed(const std::vector<VoxelCluster> &current_clusters,
                                                      const std::vector<int> &assignments,
                                                      std::map<int, ClusterTrack> &tracks,
                                                      double dt,
                                                      double speed_threshold)
{
  std::vector<VoxelCluster> result;
  for (size_t i = 0; i < current_clusters.size(); i++)
  {
    int id = assignments[i];
    if (id < 0)
    {
      result.push_back(current_clusters[i]);
      continue;
    }
    auto it = tracks.find(id);
    if (it == tracks.end())
    {
      result.push_back(current_clusters[i]);
      continue;
    }

    Point3D last_c = it->second.last_centroid;
    Point3D cur_c = calculate_cluster_centroid(current_clusters[i]);
    double dx = cur_c.x - last_c.x;
    double dy = cur_c.y - last_c.y;
    double dz = cur_c.z - last_c.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    double speed = (dt > 0.0) ? dist / dt : 0.0;

    if (speed >= speed_threshold)
    {
      result.push_back(current_clusters[i]);
    }
    it->second.last_centroid = cur_c;
  }

  return result;
}

void Clustering::calc_dynamic_cluster_indices(const std::vector<VoxelCluster> &clusters, const std::vector<VoxelCluster> &dynamic_clusters)
{
  dynamic_cluster_indices_.clear();
  for (const auto &dyn_cluster : dynamic_clusters)
  {
    Point3D dyn_center = calculate_cluster_centroid(dyn_cluster);
    for (size_t i = 0; i < clusters.size(); ++i)
    {
      Point3D orig_center = calculate_cluster_centroid(clusters[i]);
      double dx = dyn_center.x - orig_center.x;
      double dy = dyn_center.y - orig_center.y;
      double dz = dyn_center.z - orig_center.z;
      double dist_sq = dx * dx + dy * dy + dz * dz;
      double tol = 1e-6;
      if (dist_sq < tol * tol)
      {
        dynamic_cluster_indices_.insert(i);
        break;
      }
    }
  }
}

void Clustering::calc_dynamic_human_cluster_indices(const std::vector<VoxelCluster> &clusters)
{
  dynamic_human_cluster_indices_.clear();
  for (auto i : human_size_cluster_indices_)
  {
    if (dynamic_cluster_indices_.find(i) != dynamic_cluster_indices_.end())
    {
      dynamic_human_cluster_indices_.insert(i);
    }
  }
}

void Clustering::filter_dynamic_human_clusters_near_boundaries(const std::vector<VoxelCluster> &clusters)
{
  std::unordered_set<size_t> filtered_dynamic_human;
  for (auto i : dynamic_human_cluster_indices_)
  {
    Point3D centroid = calculate_cluster_centroid(clusters[i]);
    bool near_boundary =
        ((centroid.x - params_.min_x) < 2 * params_.voxel_size_x) ||
        ((params_.max_x - centroid.x) < 2 * params_.voxel_size_x) ||
        ((centroid.y - params_.min_y) < 2 * params_.voxel_size_y) ||
        ((params_.max_y - centroid.y) < 2 * params_.voxel_size_y) ||
        ((centroid.z - params_.min_z) < 2 * params_.voxel_size_z) ||
        ((params_.max_z - centroid.z) < 2 * params_.voxel_size_z);

    if (!near_boundary)
    {
      filtered_dynamic_human.insert(i);
    }
    else
    {
      // 境界付近は除外
      human_size_cluster_indices_.erase(i);
      dynamic_cluster_indices_.erase(i);
    }
  }
  dynamic_human_cluster_indices_ = filtered_dynamic_human;
}

void Clustering::refine_human_clusters(std::vector<VoxelCluster> &clusters, const Point3D &human_position)
{
  if (human_position.x == 0.0f && human_position.y == 0.0f && human_position.z == 0.0f)
  {
    return;
  }

  double min_dist = std::numeric_limits<double>::max();
  size_t best_cluster_idx = std::numeric_limits<size_t>::max();

  for (auto idx : dynamic_human_cluster_indices_)
  {
    Point3D centroid = calculate_cluster_centroid(clusters[idx]);
    double dx = centroid.x - human_position.x;
    double dy = centroid.y - human_position.y;
    double dz = centroid.z - human_position.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < min_dist)
    {
      min_dist = dist;
      best_cluster_idx = idx;
    }
  }

  if (best_cluster_idx == std::numeric_limits<size_t>::max())
  {
    return;
  }

  clusters[best_cluster_idx].points.clear();
  clusters[best_cluster_idx].points.push_back(human_position);

  std::unordered_set<size_t> refined_dynamic_human;
  refined_dynamic_human.insert(best_cluster_idx);
  dynamic_human_cluster_indices_ = refined_dynamic_human;

  std::unordered_set<size_t> refined_human_size;
  if (human_size_cluster_indices_.find(best_cluster_idx) != human_size_cluster_indices_.end())
  {
    refined_human_size.insert(best_cluster_idx);
  }
  human_size_cluster_indices_ = refined_human_size;

  std::unordered_set<size_t> refined_dynamic;
  if (dynamic_cluster_indices_.find(best_cluster_idx) != dynamic_cluster_indices_.end())
  {
    refined_dynamic.insert(best_cluster_idx);
  }
  dynamic_cluster_indices_ = refined_dynamic;
}
