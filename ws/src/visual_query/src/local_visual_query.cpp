// Implementation of the local visual-observability query (see the header).
#include "visual_query/local_visual_query.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace visual_query {

namespace {
constexpr double kEps = 1e-12;
}

// ---------------------------------------------------------------------------
// StaticOccupancyGrid
// ---------------------------------------------------------------------------
StaticOccupancyGrid::Key StaticOccupancyGrid::keyOf(const Eigen::Vector3d& p) const {
  Key k;
  k.x = static_cast<int>(std::floor(p.x() / voxel_));
  k.y = static_cast<int>(std::floor(p.y() / voxel_));
  k.z = static_cast<int>(std::floor(p.z() / voxel_));
  return k;
}

void StaticOccupancyGrid::insert(const Eigen::Vector3d& p, double w) {
  if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) return;
  if (std::fabs(p.x()) > half_ || std::fabs(p.y()) > half_ || std::fabs(p.z()) > half_) return;
  const Key k = keyOf(p);
  double& v = occ_[k];
  v = std::max(v, w);
}

void StaticOccupancyGrid::insertCloud(const std::vector<Eigen::Vector3d>& cloud) {
  for (const auto& p : cloud) insert(p);
}

double StaticOccupancyGrid::density(const Eigen::Vector3d& p) const {
  if (occ_.empty()) return 0.0;
  // Trilinear interpolation over the 2x2x2 neighbourhood so that the field is
  // continuous in p (a hard voxel lookup would put a step in the cost).
  const Eigen::Vector3d s = p / voxel_ - Eigen::Vector3d(0.5, 0.5, 0.5);
  const Eigen::Vector3d f0 = s.array().floor();
  const Eigen::Vector3d frac = s - f0;
  double acc = 0.0, wsum = 0.0;
  for (int i = 0; i < 8; ++i) {
    const int dx = (i & 1), dy = (i >> 1) & 1, dz = (i >> 2) & 1;
    Key k;
    k.x = static_cast<int>(f0.x()) + dx;
    k.y = static_cast<int>(f0.y()) + dy;
    k.z = static_cast<int>(f0.z()) + dz;
    const double w = (dx ? frac.x() : 1.0 - frac.x()) *
                     (dy ? frac.y() : 1.0 - frac.y()) *
                     (dz ? frac.z() : 1.0 - frac.z());
    auto it = occ_.find(k);
    const double v = (it == occ_.end()) ? 0.0 : it->second;
    acc += w * v;
    wsum += w;
  }
  return wsum > kEps ? acc / wsum : 0.0;
}

double StaticOccupancyGrid::transmittance(const Eigen::Vector3d& q, const Eigen::Vector3d& p,
                                         int n_samples, double absorb) const {
  if (occ_.empty()) return 1.0;
  const double L = (p - q).norm();
  if (L < kEps) return 1.0;
  n_samples = std::max(2, n_samples);
  const double ds = L / static_cast<double>(n_samples);
  double tau = 0.0;
  for (int i = 1; i <= n_samples; ++i) {
    const double a = static_cast<double>(i) / static_cast<double>(n_samples + 1);
    tau += density(q + a * (p - q)) * ds;
  }
  return std::exp(-absorb * tau);
}

// ---------------------------------------------------------------------------
// LocalVisualQuery
// ---------------------------------------------------------------------------
LocalVisualQuery::LocalVisualQuery(const QueryParams& params) : p_(params) {}

double LocalVisualQuery::sigmoid(double x) {
  if (x > 40.0) return 1.0;
  if (x < -40.0) return 0.0;
  return 1.0 / (1.0 + std::exp(-x));
}

Eigen::Vector3d LocalVisualQuery::predictCenter(const DynamicObject& o, double t) {
  return o.center + o.vel * (t - o.t_ref);
}

void LocalVisualQuery::setSnapshot(std::vector<MapPoint> pts) { pts_.swap(pts); }

std::vector<MapPoint> LocalVisualQuery::downsample(const std::vector<MapPoint>& pts,
                                                   const Eigen::Vector3d& sensor_origin,
                                                   std::size_t max_points, double voxel) {
  std::vector<std::pair<double, std::size_t>> by_dist;
  by_dist.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    by_dist.emplace_back((pts[i].p - sensor_origin).squaredNorm(), i);
  }
  std::sort(by_dist.begin(), by_dist.end());
  std::vector<MapPoint> out;
  out.reserve(std::min(max_points, pts.size()));
  // greedy voxel de-duplication: closest first, so the snapshot keeps the structure
  // the camera can actually resolve
  std::vector<Eigen::Vector3i> taken;
  taken.reserve(max_points);
  for (const auto& d : by_dist) {
    const MapPoint& mp = pts[d.second];
    const Eigen::Vector3i v(static_cast<int>(std::floor(mp.p.x() / voxel)),
                            static_cast<int>(std::floor(mp.p.y() / voxel)),
                            static_cast<int>(std::floor(mp.p.z() / voxel)));
    bool dup = false;
    for (const auto& t : taken) {
      if ((t - v).cwiseAbs().maxCoeff() <= 0) { dup = true; break; }
    }
    if (dup) continue;
    taken.push_back(v);
    out.push_back(mp);
    if (out.size() >= max_points) break;
  }
  return out;
}

bool LocalVisualQuery::project(const MapPoint& mp, const Pose2D& pose, Ray* out) const {
  const Eigen::Vector3d d = mp.p - Eigen::Vector3d(pose.x, pose.y, pose.z);
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  const double x_c = c * d.x() + s * d.y();   // forward
  const double y_c = -s * d.x() + c * d.y();  // left
  const double z_c = d.z();                   // up
  if (std::fabs(x_c) < 1e-6) return false;
  out->depth = x_c;
  out->dir_cam = Eigen::Vector3d(x_c, y_c, z_c);
  // image axes: u to the right (-left), v downwards (-up)
  out->u = cam_.cx - cam_.fx * (y_c / x_c);
  out->v = cam_.cy - cam_.fy * (z_c / x_c);
  return true;
}

double LocalVisualQuery::pointWeight(const MapPoint& mp, const Pose2D& pose, double t,
                                     double* w_fov_out, double* w_stat_out,
                                     double* w_dyn_out) const {
  Ray ray;
  if (!project(mp, pose, &ray)) {
    if (w_fov_out) *w_fov_out = 0.0;
    if (w_stat_out) *w_stat_out = 0.0;
    if (w_dyn_out) *w_dyn_out = 0.0;
    return 0.0;
  }
  const Eigen::Vector3d cam_pos(pose.x, pose.y, pose.z);
  const double r = (mp.p - cam_pos).norm();

  // --- w_fov: smooth visibility window (no hard rectangle, no hard range) -----
  const double s_front = sigmoid(ray.depth / std::max(1e-6, p_.depth_sigma));
  const double du = std::min(ray.u, static_cast<double>(cam_.width - 1) - ray.u);
  const double dv = std::min(ray.v, static_cast<double>(cam_.height - 1) - ray.v);
  const double s_border = sigmoid(du / std::max(1e-6, p_.fov_sigma_px)) *
                          sigmoid(dv / std::max(1e-6, p_.fov_sigma_px));
  const double s_range = sigmoid((r - p_.range_min) / std::max(1e-6, p_.range_sigma)) *
                         sigmoid((p_.range_max - r) / std::max(1e-6, p_.range_sigma));
  const double w_fov = s_front * s_border * s_range;

  // --- w_stat: smooth static occlusion (Beer-Lambert, no ray casting test) ---
  double w_stat = 1.0;
  if (p_.use_static_occlusion && has_grid_) {
    w_stat = grid_.transmittance(cam_pos, mp.p, 16, p_.stat_absorb);
  }

  // --- w_dyn(t): predicted 3D dynamic occlusion with constant velocity ------
  double w_dyn = 1.0;
  if (p_.use_dynamic) {
    for (const auto& o : dyn_) {
      const Eigen::Vector3d c = predictCenter(o, t);
      const Eigen::Vector3d half = (o.size * 0.5).cwiseMax(Eigen::Vector3d(1e-3, 1e-3, 1e-3));
      const Eigen::Vector3d e = (mp.p - c).cwiseQuotient(half);
      const double n = e.norm();
      // approximate signed distance to the ellipsoid surface, in metres
      const double sd = (n - 1.0) * (half.x() + half.y() + half.z()) / 3.0;
      w_dyn *= sigmoid((sd - p_.dyn_margin) / std::max(1e-6, p_.dyn_sigma));
      if (w_dyn < 1e-6) { w_dyn = 0.0; break; }
    }
  }

  if (w_fov_out) *w_fov_out = w_fov;
  if (w_stat_out) *w_stat_out = w_stat;
  if (w_dyn_out) *w_dyn_out = w_dyn;

  // w_rel is the measured reliability of this very point (never predicted)
  const double w_rel = std::min(1.0, std::max(0.0, mp.w_rel));
  return w_fov * w_stat * w_dyn * w_rel;
}

void LocalVisualQuery::buildInformation(const Pose2D& pose, double t, Eigen::Matrix3d* H_xi,
                                       SingleResult* diag) const {
  // NOTE: there is deliberately no "keep the point if weight > eps" test here.
  // Such a test makes n_used flicker and puts a step in the information whenever a
  // point crosses the threshold; the weights already go to zero smoothly outside the
  // field of view, so the contribution fades instead of switching off.
  H_xi->setZero();
  double sw = 0.0, swmin = std::numeric_limits<double>::infinity(), swmax = 0.0;
  double sdyn = 0.0, sstat = 0.0;
  int n_used = 0;
  const double c = std::cos(pose.theta), s = std::sin(pose.theta);
  for (const auto& mp : pts_) {
    double w_fov = 0.0, w_stat = 1.0, w_dyn = 1.0;
    const double w = pointWeight(mp, pose, t, &w_fov, &w_stat, &w_dyn);
    if (w <= 0.0) continue;

    Ray ray;
    if (!project(mp, pose, &ray)) continue;
    const double x_c = ray.depth;
    const double y_c = ray.dir_cam.y();
    const double z_c = ray.dir_cam.z();
    const double inv = 1.0 / x_c;
    const double inv2 = inv * inv;

    // du/d(x,y,theta) and dv/d(x,y,theta) for u = cx - fx*y_c/x_c, v = cy - fy*z_c/x_c
    const double du_dx = cam_.fx * y_c * inv2 * (-c) + (-cam_.fx * inv) * s;
    const double du_dy = cam_.fx * y_c * inv2 * (-s) + (-cam_.fx * inv) * (-c);
    const double du_dth = cam_.fx * y_c * y_c * inv2 + cam_.fx;
    const double dv_dx = cam_.fy * z_c * inv2 * (-c);
    const double dv_dy = cam_.fy * z_c * inv2 * (-s);
    const double dv_dth = cam_.fy * z_c * y_c * inv2;

    Eigen::Matrix<double, 2, 3> J;
    J << du_dx, du_dy, du_dth,
         dv_dx, dv_dy, dv_dth;
    H_xi->noalias() += w * (J.transpose() * J);

    n_used++;
    sw += w;
    swmin = std::min(swmin, w);
    swmax = std::max(swmax, w);
    // geometric means, taken in log space so that many points cannot underflow
    sdyn += std::log(std::max(1e-12, w_dyn));
    sstat += std::log(std::max(1e-12, w_stat));
  }

  if (diag) {
    diag->n_used = n_used;
    diag->n_eff = sw;   // effective number of usable points (smooth)
    diag->w_min = n_used ? swmin : 0.0;
    diag->w_max = n_used ? swmax : 0.0;
    diag->w_mean = n_used ? sw / n_used : 0.0;
    diag->w_dyn_mean = n_used ? std::exp(sdyn / n_used) : 1.0;
    diag->w_stat_mean = n_used ? std::exp(sstat / n_used) : 1.0;
  }
}

SingleResult LocalVisualQuery::query(const Pose2D& pose, double t) const {
  SingleResult out;
  Eigen::Matrix3d H_xi;
  buildInformation(pose, t, &H_xi, &out);
  if (out.n_eff < static_cast<double>(p_.min_points)) {
    out.rank_deficient = true;
    out.info = 0.0;
    out.info_raw = 0.0;
    out.worst_std = std::numeric_limits<double>::infinity();
    return out;
  }

  // Normalisation, corrected convention (task book 2.2): the physical perturbation
  // is dxi = (dx, dy, dtheta) and the required accuracy is S = diag(sigma_t,
  // sigma_t, sigma_r). Writing the perturbation in units of the requirement,
  // dxi = S z, the information matrix of z is H_z = S^T H_xi S.
  const Eigen::Vector3d s(p_.sigma_t, p_.sigma_t, p_.sigma_r);
  const Eigen::Matrix3d H_z = H_xi.cwiseProduct(s * s.transpose());

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(H_z);
  if (es.info() != Eigen::Success) {
    out.rank_deficient = true;
    out.worst_std = std::numeric_limits<double>::infinity();
    return out;
  }
  const Eigen::Vector3d evals = es.eigenvalues();
  Eigen::Vector3d evals_c = evals.cwiseMax(1e-12);
  const Eigen::Matrix3d Cn = es.eigenvectors() * evals_c.cwiseInverse().asDiagonal() *
                             es.eigenvectors().transpose();
  const Eigen::Vector3d marg = Cn.diagonal().cwiseMax(0.0).cwiseSqrt();
  const double hard = marg.maxCoeff();

  // smooth worst direction: shifted log-sum-exp soft maximum (always >= the true
  // max, hence conservative, and C^inf in H unlike a min/max over eigenvalues)
  const double tau = std::max(p_.smooth_frac * hard, 1e-12);
  double acc = 0.0;
  for (int i = 0; i < 3; ++i) acc += std::exp((marg[i] - hard) / tau);
  const double soft = tau * std::log(acc) + hard;

  out.worst_std = soft;
  out.info_raw = soft > 1e-12 ? 1.0 / soft : 0.0;
  out.info = p_.lambda_g * out.info_raw;
  return out;
}

PairResult LocalVisualQuery::query_pair(const Pose2D& a, double t_a, const Pose2D& b,
                                       double t_b, bool need_info) const {
  PairResult out;
  const Eigen::Vector3d ca(a.x, a.y, a.z), cb(b.x, b.y, b.z);
  double sum_min = 0.0, sum_a = 0.0, sum_b = 0.0, wsum = 0.0;
  double par_sum = 0.0, par_min = std::numeric_limits<double>::infinity();
  int n_cov = 0;
  for (const auto& mp : pts_) {
    const double wa = pointWeight(mp, a, t_a);
    const double wb = pointWeight(mp, b, t_b);
    sum_a += wa;
    sum_b += wb;
    const double wm = std::min(wa, wb);
    if (wm <= 0.0) continue;
    // the parallax statistics are weighted by the shared weight, so this aggregate is
    // a smooth function of both poses (a hard "seen by both" test would not be)
    sum_min += wm;
    wsum += wm;
    if (wm > 1e-3) n_cov++;
    const Eigen::Vector3d ra = (mp.p - ca).normalized();
    const Eigen::Vector3d rb = (mp.p - cb).normalized();
    const double cosang = std::min(1.0, std::max(-1.0, ra.dot(rb)));
    const double ang = std::acos(cosang) * 180.0 / M_PI;
    par_sum += wm * ang;
    if (wm > 1e-3) par_min = std::min(par_min, ang);
  }
  if (need_info)
  {
    const SingleResult ia = query(a, t_a);
    const SingleResult ib = query(b, t_b);
    out.info_a = ia.info;
    out.info_b = ib.info;
    out.n_a = ia.n_used;
    out.n_b = ib.n_used;
  }
  out.n_covis = n_cov;
  out.covis = p_.lambda_c * sum_min;
  const double weaker = std::max(kEps, std::min(sum_a, sum_b));
  out.continuity = std::min(1.0, sum_min / weaker);
  out.parallax_mean_deg = wsum > kEps ? par_sum / wsum : 0.0;
  out.parallax_min_deg = n_cov > 0 ? par_min : 0.0;
  return out;
}

}  // namespace visual_query
