/*********************************************************************
 *  EXPLORB-LOCOBS — offline self-check for the two visual-constraint g2o edges.
 *
 *  No ROS master, no Gazebo, no TEB instance: the edges only need a stub query and
 *  a TebConfig, which makes this the fastest way to answer the task book questions
 *      - weight clearing: lambda_g = lambda_c = 0 -> the native cost exactly
 *      - gradient direction and numerical stability of the custom edges
 *
 *  Build (uses the g2o/Eigen headers already on the machine):
 *    g++ -O2 -std=c++14 \
 *        -I include -I ../visual_query/include \
 *        -I /usr/include/eigen3 -I <ros include> -I <g2o include> \
 *        src/selfcheck_visual_edges.cpp -o /tmp/selfcheck_visual_edges
 *
 *  It is also built by catkin as `selfcheck_visual_edges` when visual_query is found.
 *********************************************************************/

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/edge_visual_observability.h>

using namespace teb_local_planner;

static int g_fail = 0;
static void check(const char* name, bool ok, const std::string& detail = "")
{
  std::printf("[%s] %-58s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
  if (!ok) ++g_fail;
}

// ---------------------------------------------------------------------------
// A stub query with an *analytic* shape so that the expected gradient is known:
//   g(x, y, theta) = exp(-((x-x0)^2 + (y-y0)^2) / (2 s^2)) * cos(theta - theta0)^2
// strictly smooth and bounded, with a maximum of 1 at the reference pose.
// Co-visibility falls off with distance and parallax.
// ---------------------------------------------------------------------------
class StubQuery : public IVisualQuery
{
public:
  StubQuery() : x0_(1.0), y0_(0.5), th0_(0.2), s_(0.8) {}

  double normalizedObservability(double x, double y, double theta, double /*t*/) const override
  {
    const double d2 = (x - x0_) * (x - x0_) + (y - y0_) * (y - y0_);
    const double c = std::cos(theta - th0_);
    return std::exp(-0.5 * d2 / (s_ * s_)) * c * c;
  }

  void coVisibility(double xa, double ya, double ta_, double /*ta*/,
                     double xb, double yb, double tb_, double /*tb*/,
                     double* continuity, double* parallax_min_deg) const override
  {
    const double dist = std::hypot(xb - xa, yb - ya);
    // full continuity when close, decaying when the poses drift apart
    if (continuity) *continuity = std::exp(-dist / 1.5);
    // parallax grows with the baseline, and shrinks when the headings diverge
    const double dyaw = std::fabs(tb_ - ta_);
    if (parallax_min_deg)
      *parallax_min_deg = 30.0 * dist / (1.0 + dist) * std::max(0.0, std::cos(dyaw));
  }

  bool ready() const override { return true; }

private:
  double x0_, y0_, th0_, s_;
};

// central-difference gradient of an edge error w.r.t. one scalar parameter
struct Grad
{
  double d = 0.0;
  double eps = 0.0;
};

static StubQuery g_stub;

int main()
{
  std::printf("=== selfcheck_visual_edges (no ROS, no Gazebo) ===\n");

  TebConfig cfg;   // defaults: both visual weights are 0

  // ---- 1. weight clearing -------------------------------------------------
  {
    cfg.optim.weight_visual_observability = 0.0;
    VertexPose* v = new VertexPose();
    v->setEstimate(PoseSE2(0.0, 0.0, 0.0));
    EdgeVisualObservability e;
    e.setVertex(0, v);
    e.setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e.setTebConfig(cfg);
    e.setQuery(&g_stub);               // deliberately "ready" and non-zero g
    e.setDt(0.4);
    e.setArrivalTime(0.0);
    e.computeError();
    check("lambda_g = 0 gives exactly zero cost", e.error()[0] == 0.0,
          "cost=" + std::to_string(e.error()[0]));
    delete v;
  }
  {
    cfg.optim.weight_covisibility = 0.0;
    VertexPose* a = new VertexPose();
    VertexPose* b = new VertexPose();
    a->setEstimate(PoseSE2(0.0, 0.0, 0.0));
    b->setEstimate(PoseSE2(0.6, 0.0, 0.0));
    EdgeCoVisibility e;
    e.setVertex(0, a);
    e.setVertex(1, b);
    e.setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e.setTebConfig(cfg);
    e.setQuery(&g_stub);
    e.setWeight(0.0);
    e.setTimes(0.0, 0.45);
    e.computeError();
    check("lambda_c = 0 gives exactly zero cost", e.error()[0] == 0.0,
          "cost=" + std::to_string(e.error()[0]));
    delete a;
    delete b;
  }

  // ---- 2. the unary edge actually responds to the pose -------------------
  {
    cfg.optim.weight_visual_observability = 2.0;
    const double t = 0.0;
    auto costAt = [&](double x, double y, double th) {
      VertexPose* v = new VertexPose();
      v->setEstimate(PoseSE2(x, y, th));
      EdgeVisualObservability e;
      e.setVertex(0, v);
      e.setInformation(Eigen::Matrix<double, 1, 1>::Identity());
      e.setTebConfig(cfg);
      e.setQuery(&g_stub);
      e.setDt(0.4);
      e.setArrivalTime(t);
      e.computeError();
      const double c = e.error()[0];
      delete v;
      return c;
    };

    const double c_far = costAt(0.0, 0.0, 0.0);     // bad geometry
    const double c_good = costAt(1.0, 0.5, 0.2);    // the stub's maximum
    check("cost is larger where the geometry is worse", c_far > c_good,
          "far=" + std::to_string(c_far) + " good=" + std::to_string(c_good));
    check("cost vanishes at the best pose", c_good < 1e-9,
          "good=" + std::to_string(c_good));

    // bounded: with g in [0,1] and w = lambda*dt the cost cannot exceed sqrt(w)
    const double bound = std::sqrt(2.0 * 0.4);
    check("cost is bounded by sqrt(lambda_g * dt)", c_far <= bound + 1e-12,
          "far=" + std::to_string(c_far) + " bound=" + std::to_string(bound));

    // ---- 3. gradient: finite, correct sign, and stable in epsilon --------
    double grads[2][2];
    const double eps[2] = {1e-4, 1e-5};
    for (int k = 0; k < 2; ++k)
    {
      const double h = eps[k];
      const double cx = 0.3, cy = 0.1, cth = 0.05;
      const double cp = costAt(cx + h, cy, cth);
      const double cm = costAt(cx - h, cy, cth);
      grads[k][0] = (cp - cm) / (2.0 * h);
      const double tp = costAt(cx, cy, cth + h);
      const double tm = costAt(cx, cy, cth - h);
      grads[k][1] = (tp - tm) / (2.0 * h);
    }
    check("numerical Jacobian is finite",
          std::isfinite(grads[0][0]) && std::isfinite(grads[0][1]),
          "dg/dx=" + std::to_string(grads[0][0]) + " dg/dtheta=" + std::to_string(grads[0][1]));
    check("Jacobian is stable across eps=1e-4 and 1e-5",
          std::fabs(grads[0][0] - grads[1][0]) < 1e-2 &&
          std::fabs(grads[0][1] - grads[1][1]) < 1e-2,
          "dx: " + std::to_string(grads[0][0]) + " vs " + std::to_string(grads[1][0]));

    // direction: the cost decreases towards the stub's maximum at (1, 0.5, 0.2).
    // From a pose "left" of the optimum the gradient must push x up (i.e. the cost
    // falls with increasing x, so the derivative is negative).
    check("Jacobian points towards better geometry", grads[0][0] < -1e-6,
          "dg/dx=" + std::to_string(grads[0][0]));

    // ---- 4. smoothness: no jumps along a straight sweep -----------------
    double prev = costAt(-1.0, 0.0, 0.0), max_jump = 0.0;
    for (int i = 1; i <= 400; ++i)
    {
      const double x = -1.0 + 0.005 * i;
      const double c = costAt(x, 0.0, 0.0);
      max_jump = std::max(max_jump, std::fabs(c - prev));
      prev = c;
    }
    check("cost is continuous along a 2 m sweep (no jump > 1% of bound)",
          max_jump < 0.01 * bound, "max jump=" + std::to_string(max_jump));
  }

  // ---- 5. the binary edge responds to co-visibility ----------------------
  {
    cfg.optim.weight_covisibility = 1.0;
    auto costPair = [&](double xb, double yb) {
      VertexPose* a = new VertexPose();
      VertexPose* b = new VertexPose();
      a->setEstimate(PoseSE2(0.0, 0.0, 0.0));
      b->setEstimate(PoseSE2(xb, yb, 0.0));
      EdgeCoVisibility e;
      e.setVertex(0, a);
      e.setVertex(1, b);
      e.setInformation(Eigen::Matrix<double, 1, 1>::Identity());
      e.setTebConfig(cfg);
      e.setQuery(&g_stub);
      e.setWeight(1.0);
      e.setTimes(0.0, 0.45);
      e.setParallaxRef(3.0);
      e.computeError();
      const double c = e.error()[0];
      delete a;
      delete b;
      return c;
    };
    const double c_close = costPair(0.15, 0.0);
    const double c_far = costPair(2.5, 0.0);
    check("a pair that keeps its co-visibility costs less",
          c_close < c_far,
          "close=" + std::to_string(c_close) + " far=" + std::to_string(c_far));
    check("the binary cost is bounded", c_far <= std::sqrt(1.0) * 2.0 + 1e-12,
          "far=" + std::to_string(c_far));
  }

  std::printf("=== %d failure(s) ===\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
