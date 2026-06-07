/**
 * @file object_segmentation.cpp
 * @brief Implements object segmentation for 3D point clouds.
 *
 * This file contains functions for segmenting 3D point cloud data into
 * geometric primitives (cylinders and boxes) using RANSAC and Hough transform techniques.
 * It projects 3D points onto a 2D plane, fits models, and creates collision objects for MoveIt.
 *
 * Key Features:
 *     - 3D to 2D point cloud projection
 *     - RANSAC-based line and circle fitting
 *     - Hough transform for model voting
 *     - Cylinder and box fitting to 3D point clouds
 *     - Creation of collision objects for MoveIt
 *
 * Bugs fixed (2024-10):
 *   1. DOUBLE MAPPING: fitLineRANSAC / fitCircleRANSAC were already storing
 *      original-3D indices in their inlier sets (via projection_map.at(i)).
 *      filterLineInliers / filterCircleInliers then called projection_map.at(idx)
 *      a SECOND time on those already-mapped indices, causing out-of-range lookups
 *      and silently wrong results.  Fix: RANSAC functions now store 2D indices only;
 *      the filter functions do the single, correct projection_map lookup.
 *
 *   2. INLIER REMOVAL MISMATCH: The removal set contained original-3D indices but
 *      was compared against the 2D loop counter `i`, so almost no points were ever
 *      removed.  Fix: inliers_to_remove now holds 2D projected_cloud indices.
 *      The removal loop compares `i` (2D) against that set correctly.
 *
 *   3. STALE projection_map: After shrinking projected_cloud the old projection_map
 *      mapped the new (shifted) 2D indices to wrong 3D points.  Fix: projection_map
 *      is rebuilt from scratch whenever projected_cloud is updated.
 *
 * @author Addison Sears-Collins (original), bug-fixed 2024-10
 * @date September 29, 2024
 */

#include "hello_mtc_with_perception/object_segmentation.h"

// ---------------------------------------------------------------------------
// Line fitting helpers
// ---------------------------------------------------------------------------

// Computes coefficients (a, b, c) of a 2D line  a·x + b·y + c = 0
// that passes through p1 and p2.
Eigen::Vector3f fitLine(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2) {
  Eigen::Vector3f line;
  line[0] = p2.y() - p1.y();                            // a
  line[1] = p1.x() - p2.x();                            // b
  line[2] = p2.x() * p1.y() - p1.x() * p2.y();         // c
  line.normalize();
  return line;
}

float distanceToLine(const Eigen::Vector2f& point, const Eigen::Vector3f& line) {
  return std::abs(line[0] * point.x() + line[1] * point.y() + line[2]) /
         std::sqrt(line[0] * line[0] + line[1] * line[1]);
}

// ---------------------------------------------------------------------------
// RANSAC for 2D line fitting
//
// FIX (Bug 1): inlier indices stored here are 2D indices into `cloud`
//              (i.e. the current projected_cloud).  The projection_map lookup
//              is intentionally left to the caller / filter function.
// ---------------------------------------------------------------------------
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitLineRANSAC(
    const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
    double ransac_distance_threshold,
    int ransac_max_iterations,
    const std::unordered_map<size_t, size_t>& /*projection_map — unused here now*/) {

  pcl::PointIndices::Ptr best_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr best_coefficients(new pcl::ModelCoefficients);
  best_coefficients->values.resize(3);

  if (cloud->points.size() < 2) return {best_inliers, best_coefficients};

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, static_cast<int>(cloud->points.size()) - 1);

  for (int iter = 0; iter < ransac_max_iterations; ++iter) {
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    if (idx1 == idx2) continue;

    Eigen::Vector2f p1(cloud->points[idx1].x, cloud->points[idx1].y);
    Eigen::Vector2f p2(cloud->points[idx2].x, cloud->points[idx2].y);
    Eigen::Vector3f line = fitLine(p1, p2);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector2f pt(cloud->points[i].x, cloud->points[i].y);
      if (distanceToLine(pt, line) < ransac_distance_threshold) {
        inliers->indices.push_back(static_cast<int>(i));  // ← 2D index only
      }
    }

    if (inliers->indices.size() > best_inliers->indices.size()) {
      best_inliers = inliers;
      best_coefficients->values[0] = line[0];
      best_coefficients->values[1] = line[1];
      best_coefficients->values[2] = line[2];
    }
  }

  return {best_inliers, best_coefficients};
}

// ---------------------------------------------------------------------------
// Circle fitting helpers
// ---------------------------------------------------------------------------

Eigen::Vector3f fitCircle(const Eigen::Vector2f& p1, const Eigen::Vector2f& p2,
                           const Eigen::Vector2f& p3) {
  Eigen::Matrix3f A;
  Eigen::Vector3f b;
  for (int i = 0; i < 3; ++i) {
    const Eigen::Vector2f& p = (i == 0) ? p1 : ((i == 1) ? p2 : p3);
    A.row(i) << 2 * p.x(), 2 * p.y(), 1;
    b(i) = p.x() * p.x() + p.y() * p.y();
  }
  Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);
  float radius = std::sqrt(x(0) * x(0) + x(1) * x(1) + x(2));
  return Eigen::Vector3f(x(0), x(1), radius);
}

float distanceToCircle(const Eigen::Vector2f& point, const Eigen::Vector3f& circle) {
  return std::abs((point - circle.head<2>()).norm() - circle[2]);
}

// ---------------------------------------------------------------------------
// RANSAC for 2D circle fitting
//
// FIX (Bug 1): same as fitLineRANSAC — stores 2D indices only.
// ---------------------------------------------------------------------------
std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> fitCircleRANSAC(
    const pcl::PointCloud<pcl::PointXY>::Ptr& cloud,
    double ransac_distance_threshold,
    int ransac_max_iterations,
    double max_allowable_radius,
    const std::unordered_map<size_t, size_t>& /*projection_map — unused here now*/) {

  pcl::PointIndices::Ptr best_inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr best_coefficients(new pcl::ModelCoefficients);
  best_coefficients->values.resize(3);

  if (cloud->points.size() < 3) return {best_inliers, best_coefficients};

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, static_cast<int>(cloud->points.size()) - 1);

  for (int iter = 0; iter < ransac_max_iterations; ++iter) {
    int idx1 = dis(gen);
    int idx2 = dis(gen);
    int idx3 = dis(gen);
    if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) continue;

    Eigen::Vector2f p1(cloud->points[idx1].x, cloud->points[idx1].y);
    Eigen::Vector2f p2(cloud->points[idx2].x, cloud->points[idx2].y);
    Eigen::Vector2f p3(cloud->points[idx3].x, cloud->points[idx3].y);
    Eigen::Vector3f circle = fitCircle(p1, p2, p3);

    if (circle[2] > max_allowable_radius || circle[2] <= 0) continue;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      Eigen::Vector2f pt(cloud->points[i].x, cloud->points[i].y);
      if (distanceToCircle(pt, circle) < ransac_distance_threshold) {
        inliers->indices.push_back(static_cast<int>(i));  // ← 2D index only
      }
    }

    if (inliers->indices.size() > best_inliers->indices.size()) {
      best_inliers = inliers;
      best_coefficients->values[0] = circle[0];
      best_coefficients->values[1] = circle[1];
      best_coefficients->values[2] = circle[2];
    }
  }

  return {best_inliers, best_coefficients};
}

// ---------------------------------------------------------------------------
// Logging helper
// ---------------------------------------------------------------------------
void logModelResults(const std::string& modelType,
                     const pcl::ModelCoefficients::Ptr& coefficients,
                     const pcl::PointIndices::Ptr& inliers) {
  std::ostringstream log_stream;
  if (!inliers->indices.empty()) {
    if (modelType == "Line") {
      LOG_INFO("");
      log_stream << "Line model: " << coefficients->values[0] << " x + "
                 << coefficients->values[1] << " y + "
                 << coefficients->values[2] << " = 0";
    } else if (modelType == "Circle") {
      log_stream << "Circle model: center (" << coefficients->values[0] << ", "
                 << coefficients->values[1] << "), radius " << coefficients->values[2];
    }
    LOG_INFO(log_stream.str());
    log_stream.str("");
    log_stream << modelType << " model inliers: " << inliers->indices.size();
    LOG_INFO(log_stream.str());
  } else {
    LOG_INFO("Could not estimate a " + modelType + " model for the given dataset.");
  }
}

// ---------------------------------------------------------------------------
// filterCircleInliers
//
// FIX (Bug 1): `circle_inliers` now contains 2D projected_cloud indices.
//              We do ONE projection_map lookup here to reach the original 3D point.
// ---------------------------------------------------------------------------
pcl::PointIndices::Ptr filterCircleInliers(
    const pcl::PointIndices::Ptr& circle_inliers,
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& original_cloud,
    const pcl::ModelCoefficients::Ptr& circle_coefficients,
    const std::unordered_map<size_t, size_t>& projection_map,
    int circle_min_cluster_size,
    int circle_max_clusters,
    double circle_height_tolerance,
    double circle_curvature_threshold,
    double circle_radius_tolerance,
    double circle_normal_angle_threshold,
    double circle_cluster_tolerance) {

  pcl::PointIndices::Ptr filtered_inliers(new pcl::PointIndices);

  // Build a 3D point cloud from the 2D inlier indices using a SINGLE projection_map lookup.
  // Also store the mapping from cluster-local index → original 3D index.
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<size_t> inlier_to_original;   // inlier_cloud[i] → original_cloud index

  for (const auto& idx_2d : circle_inliers->indices) {
    auto it = projection_map.find(static_cast<size_t>(idx_2d));
    if (it == projection_map.end()) continue;          // safety guard
    size_t original_idx = it->second;                 // single, correct lookup
    const auto& point = original_cloud->points[original_idx];
    inlier_cloud->points.emplace_back(point.x, point.y, point.z);
    inlier_to_original.push_back(original_idx);
  }
  inlier_cloud->width  = static_cast<uint32_t>(inlier_cloud->points.size());
  inlier_cloud->height = 1;
  inlier_cloud->is_dense = true;

  if (inlier_cloud->points.empty()) return filtered_inliers;

  // Step 1: Euclidean clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(inlier_cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(circle_cluster_tolerance);
  ec.setMinClusterSize(circle_min_cluster_size);
  ec.setMaxClusterSize(static_cast<int>(inlier_cloud->points.size()));
  ec.setSearchMethod(tree);
  ec.setInputCloud(inlier_cloud);
  ec.extract(clusters);

  if (clusters.empty()) return filtered_inliers;

  if (static_cast<int>(clusters.size()) > circle_max_clusters) return filtered_inliers;

  // Step 2: Height consistency (two clusters only)
  if (clusters.size() == 2) {
    auto max_height = [&](const pcl::PointIndices& c) {
      double h = -std::numeric_limits<double>::max();
      for (int i : c.indices)
        h = std::max(h, static_cast<double>(original_cloud->points[inlier_to_original[i]].z));
      return h;
    };
    if (std::abs(max_height(clusters[0]) - max_height(clusters[1])) > circle_height_tolerance)
      return filtered_inliers;
  }

  // Step 3: Curvature, RSD, and normal filtering using the single-mapped original 3D indices
  double circle_radius = circle_coefficients->values[2];
  Eigen::Vector3f circle_center(circle_coefficients->values[0],
                                circle_coefficients->values[1], 0.f);

  for (size_t li = 0; li < inlier_to_original.size(); ++li) {
    size_t original_idx = inlier_to_original[li];
    const auto& point = original_cloud->points[original_idx];

    if (point.curvature < circle_curvature_threshold) continue;
    if (std::abs(point.r_min - circle_radius) > circle_radius_tolerance) continue;

    Eigen::Vector3f point_vec(point.x - circle_center.x(),
                              point.y - circle_center.y(), 0.f);
    Eigen::Vector3f normal_vec(point.normal_x, point.normal_y, 0.f);
    point_vec.normalize();
    normal_vec.normalize();

    float dot = std::abs(point_vec.dot(normal_vec));
    dot = std::max(-1.f, std::min(1.f, dot));
    float angle = std::acos(dot);
    if (std::min(angle, static_cast<float>(M_PI) - angle) > circle_normal_angle_threshold)
      continue;

    filtered_inliers->indices.push_back(static_cast<int>(original_idx));
  }

  return filtered_inliers;
}

// ---------------------------------------------------------------------------
// filterLineInliers
//
// FIX (Bug 1): `line_inliers` now contains 2D projected_cloud indices.
//              We do ONE projection_map lookup here.
// ---------------------------------------------------------------------------
pcl::PointIndices::Ptr filterLineInliers(
    const pcl::PointIndices::Ptr& line_inliers,
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& original_cloud,
    const std::unordered_map<size_t, size_t>& projection_map,
    int line_min_cluster_size,
    int line_max_clusters,
    double line_curvature_threshold,
    double line_cluster_tolerance) {

  pcl::PointIndices::Ptr filtered_inliers(new pcl::PointIndices);

  // Build a 3D cloud from 2D inlier indices with ONE projection_map lookup.
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<size_t> inlier_to_original;

  for (const auto& idx_2d : line_inliers->indices) {
    auto it = projection_map.find(static_cast<size_t>(idx_2d));
    if (it == projection_map.end()) continue;
    size_t original_idx = it->second;
    const auto& point = original_cloud->points[original_idx];
    inlier_cloud->points.emplace_back(point.x, point.y, point.z);
    inlier_to_original.push_back(original_idx);
  }
  inlier_cloud->width  = static_cast<uint32_t>(inlier_cloud->points.size());
  inlier_cloud->height = 1;
  inlier_cloud->is_dense = true;

  if (inlier_cloud->points.empty()) return filtered_inliers;

  // Step 1: Euclidean clustering
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(inlier_cloud);

  std::vector<pcl::PointIndices> clusters;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(line_cluster_tolerance);
  ec.setMinClusterSize(line_min_cluster_size);
  ec.setMaxClusterSize(static_cast<int>(inlier_cloud->points.size()));
  ec.setSearchMethod(tree);
  ec.setInputCloud(inlier_cloud);
  ec.extract(clusters);

  if (static_cast<int>(clusters.size()) > line_max_clusters) return filtered_inliers;

  // Step 2: Curvature filtering
  for (size_t li = 0; li < inlier_to_original.size(); ++li) {
    size_t original_idx = inlier_to_original[li];
    const auto& point = original_cloud->points[original_idx];
    if (point.curvature > line_curvature_threshold) continue;
    filtered_inliers->indices.push_back(static_cast<int>(original_idx));
  }

  return filtered_inliers;
}

// ---------------------------------------------------------------------------
// Hough-space helpers (unchanged logic, reproduced verbatim)
// ---------------------------------------------------------------------------

static bool areBinsNeighbors(const std::vector<int>& bin1, const std::vector<int>& bin2,
                              int maxDistance = 1) {
  if (bin1.size() != bin2.size()) return false;
  for (size_t i = 0; i < bin1.size(); ++i)
    if (std::abs(bin1[i] - bin2[i]) > maxDistance) return false;
  return true;
}

std::vector<HoughBin> clusterLineModels(
    const std::vector<LineModel>& lineModels,
    double rhoThreshold,
    double thetaThreshold) {

  std::vector<HoughBin> clusters;
  std::vector<bool> processed(lineModels.size(), false);

  for (size_t i = 0; i < lineModels.size(); ++i) {
    if (processed[i]) continue;

    HoughBin newCluster;
    newCluster.votes      = lineModels[i].votes;
    newCluster.inlierCount = lineModels[i].inlierCount;
    std::vector<double> rhos   = {lineModels[i].rho};
    std::vector<double> thetas = {lineModels[i].theta};
    processed[i] = true;

    for (size_t j = i + 1; j < lineModels.size(); ++j) {
      if (processed[j]) continue;
      double rho_diff   = std::abs(lineModels[i].rho - lineModels[j].rho);
      double theta_diff = std::min(std::abs(lineModels[i].theta - lineModels[j].theta),
                                   M_PI - std::abs(lineModels[i].theta - lineModels[j].theta));
      if (rho_diff < rhoThreshold && theta_diff < thetaThreshold) {
        rhos.push_back(lineModels[j].rho);
        thetas.push_back(lineModels[j].theta);
        newCluster.votes      += lineModels[j].votes;
        newCluster.inlierCount += lineModels[j].inlierCount;
        processed[j] = true;
      }
    }

    std::sort(rhos.begin(), rhos.end());
    std::sort(thetas.begin(), thetas.end());
    size_t mid = rhos.size() / 2;
    if (rhos.size() % 2 == 0)
      newCluster.parameters = {(rhos[mid-1]+rhos[mid])/2.0, (thetas[mid-1]+thetas[mid])/2.0};
    else
      newCluster.parameters = {rhos[mid], thetas[mid]};

    clusters.push_back(newCluster);
  }

  std::sort(clusters.begin(), clusters.end(),
            [](const HoughBin& a, const HoughBin& b){ return a.votes > b.votes; });
  return clusters;
}

std::vector<HoughBin> clusterCircleHoughSpace(
    const Eigen::Tensor<int, 3>& houghSpaceCircle,
    double houghCenterXStep, double houghCenterYStep, double houghRadiusStep,
    double minPt_x, double minPt_y) {

  std::vector<HoughBin> clusters;
  std::vector<std::tuple<int,int,int,int>> voteBins;

  for (int i = 0; i < houghSpaceCircle.dimension(0); ++i)
    for (int j = 0; j < houghSpaceCircle.dimension(1); ++j)
      for (int k = 0; k < houghSpaceCircle.dimension(2); ++k)
        if (houghSpaceCircle(i,j,k) > 0)
          voteBins.emplace_back(houghSpaceCircle(i,j,k), i, j, k);

  std::sort(voteBins.begin(), voteBins.end(),
            [](const auto& a, const auto& b){ return std::get<0>(a) > std::get<0>(b); });

  std::vector<bool> processed(voteBins.size(), false);

  for (size_t m = 0; m < voteBins.size(); ++m) {
    if (processed[m]) continue;
    int votes, i, j, k;
    std::tie(votes, i, j, k) = voteBins[m];

    HoughBin newCluster;
    newCluster.indices    = {i, j, k};
    newCluster.votes      = votes;
    newCluster.parameters = {minPt_x + i*houghCenterXStep,
                             minPt_y + j*houghCenterYStep,
                             k*houghRadiusStep};
    processed[m] = true;

    for (size_t n = m + 1; n < voteBins.size(); ++n) {
      if (processed[n]) continue;
      int nv, ni, nj, nk;
      std::tie(nv, ni, nj, nk) = voteBins[n];
      if (areBinsNeighbors({i,j,k},{ni,nj,nk})) {
        double total = newCluster.votes + nv;
        newCluster.parameters[0] = (newCluster.parameters[0]*newCluster.votes +
                                    (minPt_x+ni*houghCenterXStep)*nv) / total;
        newCluster.parameters[1] = (newCluster.parameters[1]*newCluster.votes +
                                    (minPt_y+nj*houghCenterYStep)*nv) / total;
        newCluster.parameters[2] = (newCluster.parameters[2]*newCluster.votes +
                                    nk*houghRadiusStep*nv) / total;
        newCluster.votes += nv;
        processed[n] = true;
      }
    }
    clusters.push_back(newCluster);
  }
  return clusters;
}

// ---------------------------------------------------------------------------
// Shape fitting (unchanged)
// ---------------------------------------------------------------------------

std::tuple<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>
fitCylinderToCluster(
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& cluster,
    double center_x, double center_y, double radius) {

  double z_min = std::numeric_limits<double>::max();
  double z_max = -std::numeric_limits<double>::max();
  for (const auto& p : cluster->points) {
    z_min = std::min(z_min, static_cast<double>(p.z));
    z_max = std::max(z_max, static_cast<double>(p.z));
  }

  double height   = z_max - 0.0;
  double center_z = (0.0 + z_max) / 2.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[0] = height;
  primitive.dimensions[1] = radius;

  geometry_msgs::msg::Pose pose;
  pose.position.x = center_x;
  pose.position.y = center_y;
  pose.position.z = center_z;

  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return {primitive, pose};
}

double normalizeAngle(double angle) {
  while (angle > 2 * M_PI) angle -= 2 * M_PI;
  while (angle < 0)         angle += 2 * M_PI;
  return angle;
}

std::tuple<shape_msgs::msg::SolidPrimitive, geometry_msgs::msg::Pose>
fitBoxToCluster(
    const pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr& cluster,
    double rho, double theta) {

  double phi    = normalizeAngle(theta + M_PI_2);
  double along_x = std::cos(phi), along_y = std::sin(phi);
  double perp_x  = std::cos(theta), perp_y = std::sin(theta);

  double min_along =  std::numeric_limits<double>::max();
  double max_along = -std::numeric_limits<double>::max();
  double min_perp  =  std::numeric_limits<double>::max();
  double max_perp  = -std::numeric_limits<double>::max();
  double min_z     =  std::numeric_limits<double>::max();
  double max_z     = -std::numeric_limits<double>::max();

  for (const auto& p : cluster->points) {
    double along = p.x*along_x + p.y*along_y;
    double perp  = p.x*perp_x  + p.y*perp_y;
    min_along = std::min(min_along, along);
    max_along = std::max(max_along, along);
    min_perp  = std::min(min_perp,  perp);
    max_perp  = std::max(max_perp,  perp);
    min_z     = std::min(min_z, static_cast<double>(p.z));
    max_z     = std::max(max_z, static_cast<double>(p.z));
  }

  double length = max_along - min_along;
  double width  = max_perp  - min_perp;
  double height = max_z - min_z;

  double mid_along = (min_along + max_along) / 2.0;
  double center_x  = mid_along * along_x + (rho + width/2) * perp_x;
  double center_y  = mid_along * along_y + (rho + width/2) * perp_y;
  double z_position = (0.0 + max_z) / 2.0;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = length;
  primitive.dimensions[1] = width;
  primitive.dimensions[2] = height;

  geometry_msgs::msg::Pose pose;
  pose.position.x = center_x;
  pose.position.y = center_y;
  pose.position.z = z_position;

  tf2::Quaternion q;
  q.setRPY(0, 0, phi);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return {primitive, pose};
}

// ---------------------------------------------------------------------------
// segmentObjects — main entry point
//
// FIX (Bugs 2 & 3):
//   • inliers_to_remove now holds 2D projected_cloud indices so the removal
//     loop correctly skips points by their current 2D position.
//   • projection_map is rebuilt from scratch after each point-removal step
//     so subsequent RANSAC calls map the new (compacted) 2D indices correctly.
// ---------------------------------------------------------------------------
std::vector<moveit_msgs::msg::CollisionObject> segmentObjects(
    const std::vector<pcl::PointCloud<PointXYZRGBNormalRSD>::Ptr>& cloud_clusters,
    int num_iterations,
    const std::string& frame_id,
    int inlier_threshold,
    int hough_radius_bins,
    int hough_center_bins,
    double ransac_distance_threshold,
    int ransac_max_iterations,
    int circle_min_cluster_size,
    int circle_max_clusters,
    double circle_height_tolerance,
    double circle_curvature_threshold,
    double circle_radius_tolerance,
    double circle_normal_angle_threshold,
    double circle_cluster_tolerance,
    int line_min_cluster_size,
    int line_max_clusters,
    double line_curvature_threshold,
    double line_cluster_tolerance,
    double line_rho_threshold,
    double line_theta_threshold) {

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  int box_count      = 0;
  int cylinder_count = 0;
  std::ostringstream log_stream;

  // ==========================================================================
  // Outer loop — one iteration per point-cloud cluster
  // ==========================================================================
  for (const auto& cluster : cloud_clusters) {

    // -----------------------------------------------------------------------
    // Project 3D cluster onto z=0 plane; build initial projection_map
    // projection_map[2D_index] → original 3D cluster index
    // -----------------------------------------------------------------------
    pcl::PointCloud<pcl::PointXY>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXY>);
    std::unordered_map<size_t, size_t> projection_map;

    for (size_t i = 0; i < cluster->points.size(); ++i) {
      pcl::PointXY projected_point;
      projected_point.x = cluster->points[i].x;
      projected_point.y = cluster->points[i].y;
      projected_cloud->points.push_back(projected_point);
      projection_map[projected_cloud->points.size() - 1] = i;
    }
    projected_cloud->width    = static_cast<uint32_t>(projected_cloud->points.size());
    projected_cloud->height   = 1;
    projected_cloud->is_dense = true;

    // Keep an untouched copy for restoring at the end of each outer iteration
    pcl::PointCloud<pcl::PointXY>::Ptr original_projected_cloud_copy(
        new pcl::PointCloud<pcl::PointXY>);
    pcl::copyPointCloud(*projected_cloud, *original_projected_cloud_copy);

    // Also keep the initial projection_map for restoring
    const std::unordered_map<size_t, size_t> initial_projection_map = projection_map;

    log_stream.str("");
    log_stream << "\n\n\n"
               << "**********************************************************************\n"
               << "Projected " << cluster->points.size()
               << " 3D points → 2D. Cloud size: " << projected_cloud->points.size() << "\n"
               << "**********************************************************************";
    LOG_INFO(log_stream.str());

    // -----------------------------------------------------------------------
    // Hough parameter spaces
    // -----------------------------------------------------------------------
    pcl::PointXY min_pt, max_pt;
    min_pt.x = min_pt.y =  std::numeric_limits<float>::max();
    max_pt.x = max_pt.y = -std::numeric_limits<float>::max();
    for (const auto& p : projected_cloud->points) {
      min_pt.x = std::min(min_pt.x, p.x); min_pt.y = std::min(min_pt.y, p.y);
      max_pt.x = std::max(max_pt.x, p.x); max_pt.y = std::max(max_pt.y, p.y);
    }

    double projected_x_range = max_pt.x - min_pt.x;
    double projected_y_range = max_pt.y - min_pt.y;
    double hough_max_distance = std::sqrt(projected_x_range*projected_x_range +
                                          projected_y_range*projected_y_range);

    std::vector<LineModel> lineModels;

    Eigen::Tensor<int, 3> hough_space_circle(hough_center_bins, hough_center_bins, hough_radius_bins);
    hough_space_circle.setZero();

    double hough_max_radius    = hough_max_distance / 2.0;
    double hough_center_x_step = projected_x_range / hough_center_bins;
    double hough_center_y_step = projected_y_range / hough_center_bins;
    double hough_radius_step   = hough_max_radius   / hough_radius_bins;

    // ==========================================================================
    // Inner loop — repeated RANSAC iterations
    // ==========================================================================
    for (int iter = 0; iter < num_iterations; ++iter) {

      // -----------------------------------------------------------------------
      // RANSAC while-loop: keep consuming points until too few remain
      // -----------------------------------------------------------------------
      while (static_cast<int>(projected_cloud->points.size()) > inlier_threshold) {

        // Line fit — returns 2D indices
        auto [line_inliers_2d, line_coefficients] =
            fitLineRANSAC(projected_cloud, ransac_distance_threshold,
                          ransac_max_iterations, projection_map);

        // Circle fit — returns 2D indices
        auto [circle_inliers_2d, circle_coefficients] =
            fitCircleRANSAC(projected_cloud, ransac_distance_threshold,
                            ransac_max_iterations, hough_max_radius, projection_map);

        // Filter — inputs are 2D indices; outputs are original 3D indices
        pcl::PointIndices::Ptr filtered_circle_inliers = filterCircleInliers(
            circle_inliers_2d, cluster, circle_coefficients, projection_map,
            circle_min_cluster_size, circle_max_clusters, circle_height_tolerance,
            circle_curvature_threshold, circle_radius_tolerance,
            circle_normal_angle_threshold, circle_cluster_tolerance);

        pcl::PointIndices::Ptr filtered_line_inliers = filterLineInliers(
            line_inliers_2d, cluster, projection_map,
            line_min_cluster_size, line_max_clusters,
            line_curvature_threshold, line_cluster_tolerance);

        // -------------------------------------------------------------------
        // Model validation
        // -------------------------------------------------------------------
        std::vector<ValidModel> valid_models;

        if (static_cast<int>(filtered_circle_inliers->indices.size()) > inlier_threshold) {
          ValidModel m;
          m.type           = "circle";
          m.parameters     = {circle_coefficients->values[0],
                               circle_coefficients->values[1],
                               circle_coefficients->values[2]};
          m.inlier_indices = filtered_circle_inliers->indices;  // 3D indices
          valid_models.push_back(m);
        }

        if (static_cast<int>(filtered_line_inliers->indices.size()) > inlier_threshold) {
          ValidModel m;
          m.type = "line";
          double a = line_coefficients->values[0];
          double b = line_coefficients->values[1];
          double c = line_coefficients->values[2];
          double rho   = std::abs(c) / std::sqrt(a*a + b*b);
          double theta = std::atan2(b, a);
          if (theta < 0) theta += M_PI;
          m.parameters     = {rho, theta};
          m.inlier_indices = filtered_line_inliers->indices;    // 3D indices
          valid_models.push_back(m);
        }

        // Keep only the model type with more inliers when both valid
        if (!valid_models.empty()) {
          size_t best_circle = 0, best_line = 0;
          for (const auto& vm : valid_models) {
            if (vm.type == "circle") best_circle = std::max(best_circle, vm.inlier_indices.size());
            else                     best_line   = std::max(best_line,   vm.inlier_indices.size());
          }
          valid_models.erase(
              std::remove_if(valid_models.begin(), valid_models.end(),
                  [best_circle, best_line](const ValidModel& vm){
                    return (vm.type == "circle" && vm.inlier_indices.size() < best_line) ||
                           (vm.type == "line"   && vm.inlier_indices.size() < best_circle);
                  }),
              valid_models.end());
        }

        if (valid_models.empty()) break;

        // -------------------------------------------------------------------
        // Vote in Hough spaces
        // -------------------------------------------------------------------
        for (const auto& model : valid_models) {
          if (model.type == "circle") {
            int cx_bin = std::clamp(
                static_cast<int>((model.parameters[0] - min_pt.x) / hough_center_x_step),
                0, hough_center_bins - 1);
            int cy_bin = std::clamp(
                static_cast<int>((model.parameters[1] - min_pt.y) / hough_center_y_step),
                0, hough_center_bins - 1);
            int r_bin = std::clamp(
                static_cast<int>(model.parameters[2] / hough_radius_step),
                0, hough_radius_bins - 1);
            hough_space_circle(cx_bin, cy_bin, r_bin) += 1;
          } else {
            LineModel lm;
            lm.rho         = model.parameters[0];
            lm.theta       = model.parameters[1];
            lm.votes       = 1;
            lm.inlierCount = static_cast<int>(model.inlier_indices.size());
            lineModels.push_back(lm);
          }
        }

        // -------------------------------------------------------------------
        // FIX (Bug 2): Build inliers_to_remove as a set of 2D indices.
        //
        // filtered_*_inliers contain 3D (original cluster) indices.
        // We need 2D projected_cloud indices to correctly remove the right
        // points from projected_cloud.
        //
        // Strategy: scan projection_map to find all 2D indices whose 3D
        // counterpart is in the removal set.
        // -------------------------------------------------------------------
        std::set<size_t> original_3d_to_remove;
        for (const auto& vm : valid_models)
          original_3d_to_remove.insert(vm.inlier_indices.begin(), vm.inlier_indices.end());

        // Build removal set: 2D projected_cloud indices whose mapped 3D index is to be removed
        std::set<size_t> indices_2d_to_remove;
        for (const auto& kv : projection_map) {
          if (original_3d_to_remove.count(kv.second))
            indices_2d_to_remove.insert(kv.first);
        }

        // -------------------------------------------------------------------
        // Remove inlier points and rebuild projection_map for the compacted cloud
        // FIX (Bug 3): projection_map is rebuilt so new 2D indices are correct
        // -------------------------------------------------------------------
        pcl::PointCloud<pcl::PointXY>::Ptr cloud_without_inliers(
            new pcl::PointCloud<pcl::PointXY>);
        std::unordered_map<size_t, size_t> new_projection_map;
        size_t new_2d_idx = 0;

        for (size_t i = 0; i < projected_cloud->points.size(); ++i) {
          if (indices_2d_to_remove.count(i) == 0) {
            cloud_without_inliers->points.push_back(projected_cloud->points[i]);
            // Map the new 2D index → original 3D index (looked up from old map)
            auto it = projection_map.find(i);
            if (it != projection_map.end())
              new_projection_map[new_2d_idx] = it->second;
            ++new_2d_idx;
          }
        }
        cloud_without_inliers->width    = static_cast<uint32_t>(cloud_without_inliers->points.size());
        cloud_without_inliers->height   = 1;
        cloud_without_inliers->is_dense = true;

        projected_cloud  = cloud_without_inliers;
        projection_map   = new_projection_map;   // ← updated map for next RANSAC call

        LOG_INFO("Removed " + std::to_string(indices_2d_to_remove.size()) +
                 " inliers. Remaining 2D points: " +
                 std::to_string(projected_cloud->points.size()));
      }

      // Restore full 2D cloud and projection_map for the next outer iteration
      pcl::copyPointCloud(*original_projected_cloud_copy, *projected_cloud);
      projection_map = initial_projection_map;
    }

    // -----------------------------------------------------------------------
    // Cluster Hough parameter spaces and pick the winner
    // -----------------------------------------------------------------------
    std::vector<HoughBin> clusteredLineModels =
        clusterLineModels(lineModels, line_rho_threshold, line_theta_threshold);

    std::vector<HoughBin> clusteredCircleModels =
        clusterCircleHoughSpace(hough_space_circle,
                                hough_center_x_step, hough_center_y_step, hough_radius_step,
                                min_pt.x, min_pt.y);

    LOG_INFO("Clustered models:");
    LOG_INFO("  Line clusters:   " + std::to_string(clusteredLineModels.size()));
    LOG_INFO("  Circle clusters: " + std::to_string(clusteredCircleModels.size()));

    int topN = 4;
    LOG_INFO("Top line clusters:");
    for (int i = 0; i < std::min(topN, static_cast<int>(clusteredLineModels.size())); ++i) {
      const auto& c = clusteredLineModels[i];
      log_stream.str("");
      log_stream << "  Cluster " << i+1 << ": Votes=" << c.votes
                 << " Rho=" << c.parameters[0] << " Theta=" << c.parameters[1];
      LOG_INFO(log_stream.str());
    }
    LOG_INFO("Top circle clusters:");
    for (int i = 0; i < std::min(topN, static_cast<int>(clusteredCircleModels.size())); ++i) {
      const auto& c = clusteredCircleModels[i];
      log_stream.str("");
      log_stream << "  Cluster " << i+1 << ": Votes=" << c.votes
                 << " Center=(" << c.parameters[0] << "," << c.parameters[1]
                 << ") Radius=" << c.parameters[2];
      LOG_INFO(log_stream.str());
    }

    // Select winner
    std::string             top_model_type;
    std::vector<double>     top_model_parameters;
    int                     top_model_votes = 0;

    if (!clusteredLineModels.empty() && clusteredLineModels[0].votes > top_model_votes) {
      top_model_type       = "line";
      top_model_parameters = clusteredLineModels[0].parameters;
      top_model_votes      = clusteredLineModels[0].votes;
    }
    if (!clusteredCircleModels.empty() && clusteredCircleModels[0].votes > top_model_votes) {
      top_model_type       = "circle";
      top_model_parameters = clusteredCircleModels[0].parameters;
      top_model_votes      = clusteredCircleModels[0].votes;
    }

    LOG_INFO("");
    LOG_INFO("Selected top model:");
    if (top_model_type == "line") {
      LOG_INFO("  Type: Line  Votes: " + std::to_string(top_model_votes));
      LOG_INFO("  Rho: "   + std::to_string(top_model_parameters[0]));
      LOG_INFO("  Theta: " + std::to_string(top_model_parameters[1]));
    } else if (top_model_type == "circle") {
      LOG_INFO("  Type: Circle  Votes: " + std::to_string(top_model_votes));
      LOG_INFO("  Center: (" + std::to_string(top_model_parameters[0]) +
               ", " + std::to_string(top_model_parameters[1]) + ")");
      LOG_INFO("  Radius: " + std::to_string(top_model_parameters[2]));
    } else {
      LOG_INFO("  No valid model found for this cluster.");
    }
    LOG_INFO("");

    // -----------------------------------------------------------------------
    // Fit 3D shape and create CollisionObject
    // -----------------------------------------------------------------------
    if (top_model_type == "circle") {
      auto [primitive, cylinder_pose] = fitCylinderToCluster(
          cluster, top_model_parameters[0], top_model_parameters[1], top_model_parameters[2]);

      moveit_msgs::msg::CollisionObject co;
      co.header.frame_id = frame_id;
      co.id              = "cylinder_" + std::to_string(cylinder_count++);
      co.primitives.push_back(primitive);
      co.primitive_poses.push_back(cylinder_pose);
      co.operation = moveit_msgs::msg::CollisionObject::ADD;
      collision_objects.push_back(co);

      tf2::Quaternion q(cylinder_pose.orientation.x, cylinder_pose.orientation.y,
                        cylinder_pose.orientation.z, cylinder_pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      log_stream.str("");
      log_stream << "Added cylinder: id=" << co.id
                 << " height=" << primitive.dimensions[0]
                 << " radius=" << primitive.dimensions[1]
                 << " pos=(" << cylinder_pose.position.x
                 << "," << cylinder_pose.position.y
                 << "," << cylinder_pose.position.z << ")"
                 << " yaw=" << yaw;
      LOG_INFO(log_stream.str());

    } else if (top_model_type == "line") {
      auto [primitive, box_pose] = fitBoxToCluster(
          cluster, top_model_parameters[0], top_model_parameters[1]);

      moveit_msgs::msg::CollisionObject co;
      co.header.frame_id = frame_id;
      co.id              = "box_" + std::to_string(box_count++);
      co.primitives.push_back(primitive);
      co.primitive_poses.push_back(box_pose);
      co.operation = moveit_msgs::msg::CollisionObject::ADD;
      collision_objects.push_back(co);

      tf2::Quaternion q(box_pose.orientation.x, box_pose.orientation.y,
                        box_pose.orientation.z, box_pose.orientation.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw = normalizeAngle(yaw);
      log_stream.str("");
      log_stream << "Added box: id=" << co.id
                 << " dims=(" << primitive.dimensions[0]
                 << "," << primitive.dimensions[1]
                 << "," << primitive.dimensions[2] << ")"
                 << " pos=(" << box_pose.position.x
                 << "," << box_pose.position.y
                 << "," << box_pose.position.z << ")"
                 << " yaw=" << yaw;
      LOG_INFO(log_stream.str());
    }

    LOG_INFO("");
  } // end outer cluster loop

  return collision_objects;
}
