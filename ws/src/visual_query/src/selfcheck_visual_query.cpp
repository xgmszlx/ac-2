// Offline self-check for the local visual query (no ROS, no Gazebo, no master).
//
//   g++ -O2 -std=c++14 -I include -I/usr/include/eigen3 \
//       src/local_visual_query.cpp src/selfcheck_visual_query.cpp -o /tmp/vq_selfcheck
//   /tmp/vq_selfcheck
//
// These checks are the ones the task book asks for before the TEB edges may be
// trusted (phase 4.1 items 2 and 4, in their query-module form):
//   * the field is smooth (no zero-gradient plateau from hard ray/rectangle gates),
//   * lambda_g = 0 gives exactly zero information,
//   * the dynamic prediction is time- and velocity-consistent,
//   * the static occlusion gate actually attenuates a blocked line of sight,
//   * query()/query_pair() are fast enough for the TEB control period.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "visual_query/local_visual_query.h"

using namespace visual_query;

static int g_fail = 0;

static void check(const char* name, bool ok, const std::string& detail = "") {
  std::printf("[%s] %-58s %s\n", ok ? "PASS" : "FAIL", name, detail.c_str());
  if (!ok) ++g_fail;
}

static std::vector<MapPoint> makeScene(int n = 120, double z0 = 0.4, double z1 = 2.6) {
  std::vector<MapPoint> pts;
  pts.reserve(n);
  std::uint64_t id = 1;
  // a rough "corridor": two walls plus a back wall, all in front of the robot (+x)
  for (int i = 0; i < n; ++i) {
    MapPoint m;
    m.id = id++;
    const double t = static_cast<double>(i) / static_cast<double>(n - 1);
    const double x = z0 + (z1 - z0) * t;
    const int lane = i % 3;
    if (lane == 0) m.p = Eigen::Vector3d(x, 0.9, 0.5);
    else if (lane == 1) m.p = Eigen::Vector3d(x, -0.9, 0.5);
    else m.p = Eigen::Vector3d(x, 0.15 * ((i % 5) - 2), 0.4 + 0.5 * ((i % 4) / 3.0));
    pts.push_back(m);
  }
  return pts;
}

static Pose2D basePose() {
  Pose2D q;
  q.x = 0.0;
  q.y = 0.0;
  q.theta = 0.0;
  q.z = 0.30;
  return q;
}

int main() {
  std::printf("=== selfcheck_visual_query (no ROS, no Gazebo) ===\n");

  QueryParams P;
  LocalVisualQuery vq(P);
  vq.setSnapshot(makeScene());

  // ---- 1. basic sanity ----------------------------------------------------
  SingleResult r0 = vq.query(basePose(), 0.0);
  check("snapshot is used (n_used > 20)", r0.n_used > 20,
        "n_used=" + std::to_string(r0.n_used) + " info=" + std::to_string(r0.info_raw));
  check("information is finite and positive", std::isfinite(r0.info_raw) && r0.info_raw > 0.0,
        "info_raw=" + std::to_string(r0.info_raw));

  // ---- 2. smoothness (no hard gates) -------------------------------------
  // Sweep the pose across the whole field of view in 1 mm steps: a hard FOV
  // rectangle or a hard range/ray test would show up as a jump.
  {
    double prev = vq.query(basePose(), 0.0).info_raw;
    double max_step_rel = 0.0, max_theta_rel = 0.0;
    bool monotone_ok = true;
    for (int i = 1; i <= 400; ++i) {
      Pose2D q = basePose();
      q.y = 0.001 * i;  // 1 mm steps, 40 cm in total
      const double cur = vq.query(q, 0.0).info_raw;
      const double rel = std::fabs(cur - prev) / std::max(1e-9, std::fabs(prev));
      max_step_rel = std::max(max_step_rel, rel);
      if (!std::isfinite(cur)) monotone_ok = false;
      prev = cur;
    }
    prev = vq.query(basePose(), 0.0).info_raw;   // restart from the same pose
    for (int i = 1; i <= 200; ++i) {
      Pose2D q = basePose();
      q.theta = 0.002 * i;  // 0.1 deg steps, 20 deg in total
      const double cur = vq.query(q, 0.0).info_raw;
      const double rel = std::fabs(cur - prev) / std::max(1e-9, std::fabs(prev));
      max_theta_rel = std::max(max_theta_rel, rel);
      prev = cur;
    }
    check("translation sweep has no jump (max 1mm step < 2%)", max_step_rel < 0.02,
          "max step rel change=" + std::to_string(max_step_rel));
    check("rotation sweep has no jump (max 0.1deg step < 2%)", max_theta_rel < 0.02,
          "max step rel change=" + std::to_string(max_theta_rel));
    check("all evaluated values are finite", monotone_ok);
  }

  // ---- 3. the field-of-view gate is a ramp, not a rectangle --------------
  {
    // Move one point from the image centre outwards, well past the border, and check
    // that the weight decays monotonically and smoothly and never reaches exactly 0
    // (a hard rectangle would be 1 up to the border and 0 outside).
    bool mono = true, nonzero = true, smooth = true;
    double prev = 1e9, max_rel = 0.0;
    for (int px = 320; px <= 760; px += 10) {
      MapPoint m;
      m.id = 999;
      const double x = 2.0;
      // u = cx - fx*y/x  =>  y = (cx - u) * x / fx, with cx = 320, fx = 535
      const double y = (320.0 - static_cast<double>(px)) * x / 535.0;  // u = px
      m.p = Eigen::Vector3d(x, y, 0.0);
      vq.setSnapshot({m});
      double w_fov = 0.0;
      vq.pointWeight(m, basePose(), 0.0, &w_fov, nullptr, nullptr);
      if (px > 320 && w_fov > prev + 1e-12) mono = false;
      if (px > 700 && !(w_fov > 0.0)) nonzero = false;
      if (prev < 1e8) {
        const double rel = std::fabs(w_fov - prev) / std::max(1e-12, prev);
        max_rel = std::max(max_rel, rel);
        if (px > 330 && rel > 0.5) smooth = false;   // no cliff once inside the ramp
      }
      prev = w_fov;
    }
    check("fov gate decays monotonically outside the image", mono);
    check("fov gate never reaches exactly zero (no hard rays)", nonzero);
    check("fov gate is a ramp, not a cliff", smooth,
          "max 10px step rel change=" + std::to_string(max_rel));
    vq.setSnapshot(makeScene());
  }

  // ---- 4. lambda_g = 0 -> exactly zero information (weight-zero check) ----
  {
    QueryParams P0 = P;
    P0.lambda_g = 0.0;
    LocalVisualQuery vq0(P0);
    vq0.setSnapshot(makeScene());
    const SingleResult z = vq0.query(basePose(), 0.0);
    check("lambda_g = 0 gives exactly zero information", z.info == 0.0,
          "info=" + std::to_string(z.info) + " info_raw=" + std::to_string(z.info_raw));
    QueryParams P0c = P;
    P0c.lambda_c = 0.0;
    LocalVisualQuery vqc(P0c);
    vqc.setSnapshot(makeScene());
    const PairResult pc = vqc.query_pair(basePose(), 0.0, basePose(), 0.0);
    check("lambda_c = 0 gives exactly zero co-visibility cost", pc.covis == 0.0,
          "covis=" + std::to_string(pc.covis));
  }

  // ---- 5. dynamic occlusion: constant-velocity prediction ----------------
  {
    DynamicObject o;
    o.center = Eigen::Vector3d(1.5, 0.0, 0.8);
    o.size = Eigen::Vector3d(0.5, 0.5, 1.7);
    o.vel = Eigen::Vector3d(0.0, -1.0, 0.0);  // crossing to the left at 1 m/s
    o.t_ref = 0.0;
    LocalVisualQuery vq1(P);
    vq1.setSnapshot(makeScene());
    vq1.setDynamicObjects({o});
    const SingleResult blocked = vq1.query(basePose(), 0.0);
    check("an object in front reduces the dynamic gate",
          blocked.w_dyn_mean < 0.95,
          "w_dyn_mean=" + std::to_string(blocked.w_dyn_mean));
    {
      // one point straight behind the object: its weight must collapse
      LocalVisualQuery vqs(P);
      MapPoint behind;
      behind.id = 1;
      behind.p = Eigen::Vector3d(1.5, 0.0, 0.8);   // exactly the object centre
      vqs.setSnapshot({behind});
      vqs.setDynamicObjects({o});
      double w_dyn = 1.0;
      vqs.pointWeight(behind, basePose(), 0.0, nullptr, nullptr, &w_dyn);
      check("a point at the object centre is strongly attenuated", w_dyn < 0.05,
            "w_dyn=" + std::to_string(w_dyn));
    }
    // 3 s later the object has moved 3 m away -> the line of sight must be clear
    const SingleResult cleared = vq1.query(basePose(), 3.0);
    check("constant-velocity prediction clears the occlusion after 3 s",
          cleared.w_dyn_mean > 0.99,
          "w_dyn_mean(t=3s)=" + std::to_string(cleared.w_dyn_mean));
    // and the information must rise again when the view clears
    check("cleared view has more information than the blocked one",
          cleared.info_raw > blocked.info_raw,
          "blocked=" + std::to_string(blocked.info_raw) +
              " cleared=" + std::to_string(cleared.info_raw));
    // the prediction gate must be smooth in time as well
    double prev = vq1.query(basePose(), 0.0).w_dyn_mean, max_rel = 0.0;
    for (int i = 1; i <= 300; ++i) {
      const double cur = vq1.query(basePose(), 0.01 * i).w_dyn_mean;
      max_rel = std::max(max_rel, std::fabs(cur - prev) / std::max(1e-9, std::fabs(prev)));
      prev = cur;
    }
    check("dynamic gate is smooth in time (10 ms step < 10%)", max_rel < 0.10,
          "max rel change=" + std::to_string(max_rel));

    // disabling the prediction must restore the static-only behaviour (ablation)
    QueryParams Pnd = P;
    Pnd.use_dynamic = false;
    LocalVisualQuery vqnd(Pnd);
    vqnd.setSnapshot(makeScene());
    vqnd.setDynamicObjects({o});
    const SingleResult nd = vqnd.query(basePose(), 0.0);
    check("use_dynamic=false restores the dynamic gate to 1", nd.w_dyn_mean > 0.999,
          "w_dyn_mean=" + std::to_string(nd.w_dyn_mean));
  }

  // ---- 6. static occlusion grid -----------------------------------------
  {
    StaticOccupancyGrid g(0.20, 8.0);
    std::vector<Eigen::Vector3d> wall;
    for (double y = -1.2; y <= 1.2; y += 0.15)
      for (double z = 0.0; z <= 1.6; z += 0.15) wall.emplace_back(1.0, y, z);
    g.insertCloud(wall);
    LocalVisualQuery vq2(P);
    vq2.setSnapshot(makeScene());
    vq2.setStaticGrid(g);
    const SingleResult occl = vq2.query(basePose(), 0.0);
    check("a static wall attenuates the static gate", occl.w_stat_mean < 0.7,
          "w_stat_mean=" + std::to_string(occl.w_stat_mean) + " voxels=" +
              std::to_string(g.occupiedVoxels()));
    LocalVisualQuery vq3(P);
    vq3.setSnapshot(makeScene());
    const SingleResult clear = vq3.query(basePose(), 0.0);
    check("no grid -> static gate is exactly 1", clear.w_stat_mean > 0.999,
          "w_stat_mean=" + std::to_string(clear.w_stat_mean));
    check("the wall reduces the total weight",
          occl.w_mean < clear.w_mean,
          "occluded=" + std::to_string(occl.w_mean) + " clear=" + std::to_string(clear.w_mean));
  }

  // ---- 7. query_pair: co-visibility and parallax ------------------------
  {
    LocalVisualQuery vqp(P);
    vqp.setSnapshot(makeScene());
    const PairResult same = vqp.query_pair(basePose(), 0.0, basePose(), 0.0);
    check("identical poses -> zero parallax, full continuity",
          same.parallax_mean_deg < 1e-6 && same.continuity > 0.999 && same.covis > 0.0,
          "parallax=" + std::to_string(same.parallax_mean_deg) +
              " continuity=" + std::to_string(same.continuity));

    Pose2D moved = basePose();
    moved.x = 0.4;
    moved.theta = 0.15;
    const PairResult pair = vqp.query_pair(basePose(), 0.0, moved, 0.4);
    check("a displaced pose has real parallax",
          pair.parallax_mean_deg > 3.0,
          "parallax_mean=" + std::to_string(pair.parallax_mean_deg) +
              " min=" + std::to_string(pair.parallax_min_deg) + " deg");
    check("co-visibility is reported and bounded",
          pair.n_covis > 10 && pair.covis <= same.covis + 1e-9 && pair.continuity <= 1.0,
          "n_covis=" + std::to_string(pair.n_covis) + " covis=" + std::to_string(pair.covis));

    // facing away must lose the shared constraint
    Pose2D away = basePose();
    away.theta = M_PI;
    const PairResult lost = vqp.query_pair(basePose(), 0.0, away, 0.0);
    check("a pose facing away loses the co-visibility",
          lost.n_covis < same.n_covis / 4,
          "n_covis=" + std::to_string(lost.n_covis) + " (vs " +
              std::to_string(same.n_covis) + ")");
  }

  // ---- 8. downsampling --------------------------------------------------
  {
    std::vector<MapPoint> big;
    for (int i = 0; i < 4000; ++i) {
      MapPoint m;
      m.id = static_cast<std::uint64_t>(i);
      // spread over a 6 m x 6 m x 2.4 m volume: thousands of distinct 0.35 m voxels
      m.p = Eigen::Vector3d(0.5 + 6.0 * ((i % 50) / 49.0),
                            -3.0 + 6.0 * (((i / 50) % 20) / 19.0),
                            0.3 + 2.0 * (((i / 1000) % 4) / 3.0));
      big.push_back(m);
    }
    const auto small = LocalVisualQuery::downsample(big, Eigen::Vector3d::Zero(), 120, 0.35);
    check("downsample bounds the snapshot to 80-150 points",
          small.size() >= 80 && small.size() <= 150,
          "kept=" + std::to_string(small.size()) + " of " + std::to_string(big.size()));
  }

  // ---- 9. timing --------------------------------------------------------
  {
    LocalVisualQuery vqt(P);
    vqt.setSnapshot(makeScene());
    Pose2D q = basePose();
    const int N = 2000;
    auto t0 = std::chrono::high_resolution_clock::now();
    double acc = 0.0;
    for (int i = 0; i < N; ++i) {
      q.theta = 0.001 * i;
      acc += vqt.query(q, 0.01 * i).info_raw;
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    const double q_us = std::chrono::duration<double, std::micro>(t1 - t0).count() / N;
    auto t2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; ++i) {
      q.theta = 0.001 * i;
      acc += vqt.query_pair(q, 0.01 * i, q, 0.02 * i).covis;
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    const double p_us = std::chrono::duration<double, std::micro>(t3 - t2).count() / N;
    std::printf("       query() = %.1f us, query_pair() = %.1f us (120-point snapshot, acc=%g)\n",
                q_us, p_us, acc);
    check("query() well inside a 100 ms TEB cycle", q_us < 2000.0,
          std::to_string(q_us) + " us");
    check("query_pair() well inside a 100 ms TEB cycle", p_us < 4000.0,
          std::to_string(p_us) + " us");
  }

  // ---- 10. rank deficiency ---------------------------------------------
  {
    LocalVisualQuery vqr(P);
    vqr.setSnapshot({makeScene().front()});
    const SingleResult one = vqr.query(basePose(), 0.0);
    check("too few points -> rank deficient, zero information",
          one.rank_deficient && one.info == 0.0,
          "n_used=" + std::to_string(one.n_used));
  }

  std::printf("=== %d failure(s) ===\n", g_fail);
  return g_fail == 0 ? 0 : 1;
}
