// Local visual-observability query for the TEB local trajectory optimiser.
//
// Why this exists (task book, phase 2.3)
// -------------------------------------
// The candidate scoring that used to live in Python (`loo_observability.py`) is a
// static, whole-map, once-per-decision evaluation of a move_base plan. A TEB inner
// iteration needs the same quantity for *every* pose vertex, many times per second,
// with a small, bounded amount of work. Crossing the Python boundary there would
// dominate the control period, so this module is pure C++ (Eigen only, no ROS types
// in the core) and can be unit-checked without a simulator:
//
//     g++ -O2 -std=c++14 -I include -I/usr/include/eigen3 \
//         src/local_visual_query.cpp src/selfcheck_visual_query.cpp -o /tmp/vq_selfcheck
//     /tmp/vq_selfcheck
//
// What is queried
// ---------------
//   query(pose, t)                     -> normalised geometric information of one
//                                         hypothetical base pose at time t
//   query_pair(pose_a, t_a, pose_b, t_b) -> co-visibility (shared map points) and the
//                                         parallax change between two poses, in the
//                                         spirit of APACE (parallax_util.cpp)
//
// The map is a *downsampled snapshot* of 80-150 representative static map points,
// refreshed by the caller on the outer TEB loop. Nothing here touches the SLAM
// frontend, its feature budget, the keyframe policy or the BA: it is read-only
// geometry on a snapshot.
//
// Differentiability
// -----------------
// Every gate is a smooth sigmoid, never a hard ray/threshold: the field-of-view
// border, the range window, the signed distance to a predicted dynamic ellipsoid and
// the static-occlusion transmittance. A hard gate would give exactly zero gradient
// everywhere inside the kept region, which is the failure mode the task book calls
// out. The weights are all in (0,1] and multiply into the information matrix, so the
// cost is C^1 in the pose.
#ifndef VISUAL_QUERY_LOCAL_VISUAL_QUERY_H
#define VISUAL_QUERY_LOCAL_VISUAL_QUERY_H

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace visual_query {

// ---------------------------------------------------------------------------
// inputs
// ---------------------------------------------------------------------------

/// One representative static map point of the snapshot.
struct MapPoint {
  std::uint64_t id = 0;
  Eigen::Vector3d p = Eigen::Vector3d::Zero();  // map frame (m)
  /// Measured empirical observation reliability w_rel of *this* point, in [0,1]
  /// (n_real / n_exp over its observations). NOT a dynamic occlusion probability:
  /// see ws/src/decision_maker/src/python/loo_occlusion.py for the argument.
  double w_rel = 1.0;
};

/// A tracked dynamic object with constant-velocity prediction.
struct DynamicObject {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // map frame, valid at t_ref
  Eigen::Vector3d size = Eigen::Vector3d(0.4, 0.4, 1.6);  // full extents (m)
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();          // m/s
  double t_ref = 0.0;                                     // s (sim or ros time)
};

/// Planar base pose, the same DOF as a TEB pose vertex.
struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;  // rad, heading of the camera optical axis in the map frame
  double z = 0.30;     // camera height above the map plane (m)
};

struct CameraIntrinsics {
  double fx = 535.0, fy = 535.0, cx = 320.0, cy = 240.0;
  int width = 640, height = 480;
};

struct QueryParams {
  // smooth gates
  double fov_sigma_px = 24.0;    // border softness (pixels)
  double range_min = 0.15;       // m
  double range_max = 3.0;        // m
  double range_sigma = 0.20;     // m, softness of both range gates
  double depth_sigma = 0.10;     // m, keeps points behind the camera at ~0
  double dyn_sigma = 0.12;       // m, softness of the dynamic-occlusion gate
  double dyn_margin = 0.10;      // m, extra clearance around a predicted object
  double stat_absorb = 2.5;      // 1/m, Beer-Lambert absorption of the static grid
  double stat_sigma = 0.10;      // m, softness of the occupancy density
  // required accuracy -> the S matrix of the normalisation (see the .cpp)
  double sigma_t = 0.05;  // m
  double sigma_r = 0.02;  // rad
  double smooth_frac = 0.1;  // temperature of the smooth worst-direction metric
  int min_points = 12;       // below this the pose is rank deficient
  // switches (ablations / weight-zero check)
  bool use_dynamic = true;
  bool use_static_occlusion = true;
  int  max_snapshot = 100;   // points kept in the local snapshot (task book: 80-150)
  double lambda_g = 1.0;  // scales the whole single-pose information term
  double lambda_c = 1.0;  // scales the whole co-visibility term
};

// ---------------------------------------------------------------------------
// static occupancy (built from the octomap/gridmapper snapshot, read-only)
// ---------------------------------------------------------------------------
class StaticOccupancyGrid {
 public:
  StaticOccupancyGrid(double voxel = 0.20, double half_extent = 8.0)
      : voxel_(voxel), half_(half_extent) {}

  void clear() { occ_.clear(); }
  void insert(const Eigen::Vector3d& p, double w = 1.0);
  void insertCloud(const std::vector<Eigen::Vector3d>& cloud);

  /// Smooth occupancy density at a point, in [0,1] (trilinear over the voxel grid).
  double density(const Eigen::Vector3d& p) const;

  /// Beer-Lambert transmittance along q -> p: 1 = clear line of sight,
  /// 0 = fully blocked. Smooth in q because both the sample positions and the
  /// density field are smooth.
  double transmittance(const Eigen::Vector3d& q, const Eigen::Vector3d& p,
                       int n_samples = 16, double absorb = 2.5) const;

  std::size_t occupiedVoxels() const { return occ_.size(); }
  double voxelSize() const { return voxel_; }

 private:
  struct Key {
    int x, y, z;
    bool operator==(const Key& o) const { return x == o.x && y == o.y && z == o.z; }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const {
      return (static_cast<std::size_t>(k.x) * 73856093u) ^
             (static_cast<std::size_t>(k.y) * 19349663u) ^
             (static_cast<std::size_t>(k.z) * 83492791u);
    }
  };
  Key keyOf(const Eigen::Vector3d& p) const;
  double voxel_, half_;
  std::unordered_map<Key, double, KeyHash> occ_;
};

// ---------------------------------------------------------------------------
// results
// ---------------------------------------------------------------------------
struct SingleResult {
  double info = 0.0;       // lambda_g * sqrt-scaled worst normalised information
  double info_raw = 0.0;   // the same before lambda_g
  double worst_std = 0.0;  // the smooth worst normalised marginal std (1/info)
  int n_used = 0;          // points with a positive weight
  double n_eff = 0.0;      // effective number of usable points = sum of weights
  double w_min = 0.0, w_mean = 0.0, w_max = 0.0;  // weight distribution diagnostics
  double w_dyn_mean = 1.0;      // mean dynamic gate (1 = nothing in the way)
  double w_stat_mean = 1.0;     // mean static transmittance
  bool rank_deficient = false;
};

struct PairResult {
  double covis = 0.0;         // lambda_c * sum_i min(w_a, w_b)   (shared, weighted)
  int n_covis = 0;            // points seen by both poses
  int n_a = 0, n_b = 0;
  double parallax_mean_deg = 0.0;
  double parallax_min_deg = 0.0;  // the weakest shared baseline (the APACE-flavoured part)
  double info_a = 0.0, info_b = 0.0;
  /// Co-visibility continuity in [0,1]: 1 = the pair keeps (almost) all of the
  /// weaker pose's points, 0 = the shared constraint is lost.
  double continuity = 0.0;
};

// ---------------------------------------------------------------------------
// the module
// ---------------------------------------------------------------------------
class LocalVisualQuery {
 public:
  explicit LocalVisualQuery(const QueryParams& params = QueryParams());

  void setParams(const QueryParams& p) { p_ = p; }
  const QueryParams& params() const { return p_; }
  void setCamera(const CameraIntrinsics& c) { cam_ = c; }
  const CameraIntrinsics& camera() const { return cam_; }

  /// Replace the snapshot. The caller supplies 80-150 representative points; the
  /// helper below does the downsampling when a raw cloud is all that is available.
  void setSnapshot(std::vector<MapPoint> pts);

  /// Greedy voxel downsampling that keeps the points closest to the sensor first,
  /// so the snapshot is representative of what the camera can actually use.
  static std::vector<MapPoint> downsample(const std::vector<MapPoint>& pts,
                                          const Eigen::Vector3d& sensor_origin,
                                          std::size_t max_points = 120,
                                          double voxel = 0.35);

  void setStaticGrid(const StaticOccupancyGrid& g) { grid_ = g; has_grid_ = true; }
  void clearStaticGrid() { has_grid_ = false; }
  void setDynamicObjects(const std::vector<DynamicObject>& objs) { dyn_ = objs; }

  std::size_t snapshotSize() const { return pts_.size(); }

  // ---- the two required entry points -------------------------------------
  SingleResult query(const Pose2D& pose, double t) const;
  PairResult query_pair(const Pose2D& a, double t_a, const Pose2D& b, double t_b) const;

  // ---- exposed pieces, used by the self-checks and by the debug logs ------
  double pointWeight(const MapPoint& mp, const Pose2D& pose, double t,
                     double* w_fov = nullptr, double* w_stat = nullptr,
                     double* w_dyn = nullptr) const;
  static double sigmoid(double x);
  static Eigen::Vector3d predictCenter(const DynamicObject& o, double t);

 private:
  struct Ray {
    Eigen::Vector3d dir_cam;  // point direction in the camera frame
    double depth = 0.0;
    double u = 0.0, v = 0.0;  // pixel coordinates (may be outside the image)
  };
  bool project(const MapPoint& mp, const Pose2D& pose, Ray* out) const;
  void buildInformation(const Pose2D& pose, double t, Eigen::Matrix3d* H_xi,
                        SingleResult* diag) const;

  QueryParams p_;
  CameraIntrinsics cam_;
  std::vector<MapPoint> pts_;
  std::vector<DynamicObject> dyn_;
  StaticOccupancyGrid grid_{0.2, 8.0};
  bool has_grid_ = false;
};

}  // namespace visual_query

#endif  // VISUAL_QUERY_LOCAL_VISUAL_QUERY_H
