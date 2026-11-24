/*********************************************************************
 *  EXPLORB-LOCOBS — TEB visual-constraint edges.
 *
 *  Two bounded soft costs are added to the *existing* TEB g2o graph so that the
 *  optimiser itself trades "stay where the camera keeps a good geometric
 *  constraint" against the native terms (obstacles, kinematics, velocities,
 *  accelerations, transition time). Nothing here is a hard rejection: every term is
 *  a smooth, bounded function of the pose, so a forward trajectory is never
 *  forbidden just because the visual geometry is poor.
 *
 *    EdgeVisualObservability  (unary,  on VertexPose)     penalises a pose whose
 *                             normalised geometric information is low, weighted by
 *                             the time the robot would spend there (dt).
 *    EdgeCoVisibility         (binary, on two VertexPoses) penalises losing the
 *                             co-visibility between two poses that are one fixed
 *                             perceptual interval apart, and penalises a pair whose
 *                             parallax is too small to triangulate.
 *
 *  Both read a *shared, externally refreshed* query object (IVisualQuery), so the
 *  band's dynamic-object predictions are updated on the TEB outer loop without
 *  touching the inner optimisation.
 *
 *  Jacobians: the first version deliberately relies on g2o's numerical
 *  differentiation (`linearizeOplus()` of the base classes), because a wrong
 *  hand-written Jacobian here is much more expensive than a slower one. The
 *  self-check logs the gradient direction and stability.
 *
 *  Reference: teb_local_planner's own edge_dynamic_obstacle.h (unary pose edge with
 *  a time-parameterised obstacle prediction) and APACE's parallax/covisibility
 *  utility.
 *********************************************************************/

#ifndef _EDGE_VISUAL_OBSERVABILITY_H_
#define _EDGE_VISUAL_OBSERVABILITY_H_

#include <cmath>

#include <Eigen/Core>

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{

/**
 * @class IVisualQuery
 * @brief Abstract access to the local visual-observability query (see the
 *        visual_query package). Kept abstract so the edges do not depend on the
 *        query implementation and can be unit-checked with a stub.
 *
 * All three methods must be smooth (C^1) in the pose arguments: they are used
 * inside a gradient-based optimiser.
 */
class IVisualQuery
{
public:
  virtual ~IVisualQuery() {}

  /// Normalised single-pose observability in [0, 1]; 1 = the strongest constraint the
  /// scene can offer at the reference scale, 0 = no usable geometric information.
  virtual double normalizedObservability(double x, double y, double theta, double t) const = 0;

  /// Co-visibility continuity in [0,1] (1 = the pair keeps all of the weaker pose's
  /// usable map points) and the weakest shared parallax in degrees.
  virtual void coVisibility(double xa, double ya, double ta_, double ta,
                           double xb, double yb, double tb_, double tb,
                           double* continuity, double* parallax_min_deg) const = 0;

  /// True when the query has a map snapshot to work on.
  virtual bool ready() const = 0;
};

/**
 * @class VisualQueryRegistry
 * @brief Process-wide handle to the (externally owned) visual query.
 *
 * TEB builds one TebOptimalPlanner per homotopy class, and the ROS layer that owns the
 * query does not get to poke into each of them. A tiny registry keeps the two decoupled:
 * the ROS layer publishes "this is the query and this is the absolute time of the first
 * vertex" once per outer planning cycle, and every optimizer that builds its graph
 * reads it. The pointer is not owned here.
 */
class VisualQueryRegistry
{
public:
  static const IVisualQuery*& instance()
  {
    static const IVisualQuery* q = nullptr;
    return q;
  }
  static double& timeOrigin()
  {
    static double t = 0.0;
    return t;
  }
  static void set(const IVisualQuery* q) { instance() = q; }
  static const IVisualQuery* get() { return instance(); }
  static void setTimeOrigin(double t) { timeOrigin() = t; }
  static double getTimeOrigin() { return timeOrigin(); }
};

/**
 * @class EdgeVisualObservability
 * @brief Unary bounded cost on a single pose: penalise poor visual geometry,
 *        proportionally to the exposure time.
 *
 * error = sqrt(w_i) * (1 - g_i),  w_i = weight_visual_observability * dt_i
 * The squared error therefore costs w_i * (1-g_i)^2, which is exactly the
 * "information deficit times exposure" term evaluated at a single vertex.
 */
class EdgeVisualObservability : public BaseTebUnaryEdge<1, double, VertexPose>
{
public:
  EdgeVisualObservability() : dt_(0.0), t_arrival_(0.0), query_(nullptr) {}

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeVisualObservability()");
    const VertexPose* pose = static_cast<const VertexPose*>(_vertices[0]);
    if (query_ == nullptr || !query_->ready())
    {
      _error[0] = 0.0;
      return;
    }
    const double g = query_->normalizedObservability(pose->x(), pose->y(), pose->theta(), t_arrival_);
    const double deficit = std::min(1.0, std::max(0.0, 1.0 - g));
    const double w = std::max(0.0, cfg_->optim.weight_visual_observability * dt_);
    _error[0] = std::sqrt(w) * deficit;
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeVisualObservability::computeError() _error[0]=%f\n",
                   _error[0]);
  }

  // NOTE: no linearizeOplus() on purpose -> g2o's numerical differentiation is used
  // (the base class default). Task book phase 3.2 asks for exactly this.

  void setQuery(const IVisualQuery* q) { query_ = q; }
  void setDt(double dt) { dt_ = dt; }
  void setArrivalTime(double t) { t_arrival_ = t; }

  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

protected:
  double dt_;          ///< time the robot spends at this vertex (s)
  double t_arrival_;   ///< absolute time at which this vertex is reached (s)
  const IVisualQuery* query_;
};

/**
 * @class EdgeCoVisibility
 * @brief Binary bounded cost: keep the co-visibility between two poses that are a
 *        fixed perceptual interval apart, and keep enough parallax to triangulate.
 *
 * error = sqrt(w) * ( (1 - continuity) + max(0, 1 - parallax_min / parallax_ref) )
 *        w = weight_covisibility * (dt_a + dt_b) / 2
 * Both summands are in [0,1], so the edge is bounded; if continuity is perfect and
 * the parallax is at least the reference the error is exactly zero.
 */
class EdgeCoVisibility : public BaseTebBinaryEdge<1, double, VertexPose, VertexPose>
{
public:
  EdgeCoVisibility() : w_(0.0), ta_(0.0), tb_(0.0), parallax_ref_deg_(3.0), query_(nullptr) {}

  void computeError()
  {
    ROS_ASSERT_MSG(cfg_, "You must call setTebConfig on EdgeCoVisibility()");
    const VertexPose* a = static_cast<const VertexPose*>(_vertices[0]);
    const VertexPose* b = static_cast<const VertexPose*>(_vertices[1]);
    if (query_ == nullptr || !query_->ready())
    {
      _error[0] = 0.0;
      return;
    }
    double continuity = 1.0, parallax_min = 180.0;
    query_->coVisibility(a->x(), a->y(), a->theta(), ta_,
                         b->x(), b->y(), b->theta(), tb_,
                         &continuity, &parallax_min);
    const double cov_gap = std::min(1.0, std::max(0.0, 1.0 - continuity));
    const double par_gap = std::min(1.0, std::max(0.0, 1.0 - parallax_min / std::max(1e-6, parallax_ref_deg_)));
    _error[0] = std::sqrt(std::max(0.0, w_)) * (cov_gap + par_gap);
    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeCoVisibility::computeError() _error[0]=%f\n", _error[0]);
  }

  void setQuery(const IVisualQuery* q) { query_ = q; }
  void setWeight(double w) { w_ = w; }
  void setTimes(double ta, double tb) { ta_ = ta; tb_ = tb; }
  void setParallaxRef(double deg) { parallax_ref_deg_ = deg; }

  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

protected:
  double w_;
  double ta_, tb_;
  double parallax_ref_deg_;
  const IVisualQuery* query_;
};

} // namespace teb_local_planner

#endif // _EDGE_VISUAL_OBSERVABILITY_H_
