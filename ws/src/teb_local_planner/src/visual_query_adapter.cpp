/*********************************************************************
 *  EXPLORB-LOCOBS — implementation of the visual_query adapter.
 *********************************************************************/

#include <teb_local_planner/visual_query_adapter.h>

#ifdef TEB_HAS_VISUAL_QUERY

#include <algorithm>
#include <cmath>

#include <sensor_msgs/point_cloud2_iterator.h>

namespace teb_local_planner
{

VisualQueryAdapter::VisualQueryAdapter()
{
  // Tunables that describe *this* platform's camera and the required accuracy of the
  // localisation. The weights themselves live in the TEB config.
  params_.sigma_t = 0.05;      // m
  params_.sigma_r = 0.02;      // rad
  params_.range_max = 3.0;     // m, same order as the SLAM frontend's usable range
  params_.lambda_g = 1.0;
  params_.lambda_c = 1.0;
  params_.max_snapshot = 100;   // 80-150 representative points, per the task book
}

VisualQueryAdapter::~VisualQueryAdapter() = default;

void VisualQueryAdapter::initialize(ros::NodeHandle& nh, const std::string& map_points_topic,
                                   const std::string& occupancy_topic,
                                   const std::string& objects_topic)
{
  // resolve and log the *actual* topic names: a silently wrong namespace is exactly how
  // the visual edges ended up being skipped in every arm of the first A/B round.
  const std::string mp = ros::names::resolve(map_points_topic);
  const std::string oc = ros::names::resolve(occupancy_topic);
  const std::string ob = ros::names::resolve(objects_topic);
  sub_points_ = nh.subscribe(mp, 1, &VisualQueryAdapter::pointsCallback, this);
  sub_occ_ = nh.subscribe(oc, 1, &VisualQueryAdapter::occupancyCallback, this);
  bool use_dynamic = true;
  nh.param("visual_use_dynamic", use_dynamic, use_dynamic);
  params_.use_dynamic = use_dynamic;

  // The static-occlusion term is measured (not assumed) to be the dominant cost of a
  // query: 16 trilinear density samples per map point, per numeric-Jacobian evaluation.
  // It also duplicates information TEB already has from the costmap, so it is OFF by
  // default and can be switched on for the diagnostic runs.
  bool use_static = false;
  nh.param("visual_use_static_occlusion", use_static, use_static);
  params_.use_static_occlusion = use_static;
  ROS_INFO_STREAM("[visual_query] static-occlusion term " << (use_static ? "ON" : "OFF")
                  << ", dynamic-occlusion term " << (use_dynamic ? "ON" : "OFF")
                  << ", snapshot " << params_.max_snapshot << " points");
  if (use_dynamic)
    sub_objs_ = nh.subscribe(ob, 2, &VisualQueryAdapter::objectsCallback, this);
  else
    ROS_INFO_STREAM("[visual_query] dynamic occlusion prediction DISABLED for this arm");
  ROS_INFO_STREAM("[visual_query] adapter subscribed to " << mp << ", " << oc << ", " << ob);
}

void VisualQueryAdapter::pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  std::vector<visual_query::MapPoint> pts;
  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    pts.reserve(msg->width * msg->height);
    std::uint64_t id = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      ++id;
      const Eigen::Vector3d p(*it_x, *it_y, *it_z);
      if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
        continue;
      if (p.cwiseAbs().maxCoeff() < 1e-6)
        continue;                                  // un-filled slot of the cloud
      visual_query::MapPoint mp;
      mp.id = id;
      mp.p = p;
      mp.w_rel = 1.0;                              // measurement reliability applied on the decision side
      pts.push_back(mp);
    }
  }
  catch (const std::exception& e)
  {
    ROS_WARN_STREAM_THROTTLE(5.0, "[visual_query] cannot parse map_points: " << e.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  raw_points_.swap(pts);
  have_points_ = !raw_points_.empty();
}

void VisualQueryAdapter::occupancyCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (msg->info.width < 2 || msg->info.height < 2 || msg->data.empty())
    return;

  const double res = msg->info.resolution;
  const double ox = msg->info.origin.position.x;
  const double oy = msg->info.origin.position.y;
  visual_query::StaticOccupancyGrid grid(0.20, 10.0);

  // 2.5D: an occupied cell becomes a short vertical column, which is what the camera
  // actually sees. Smooth density comes from the query's trilinear interpolation.
  for (unsigned int y = 0; y < msg->info.height; ++y)
  {
    for (unsigned int x = 0; x < msg->info.width; ++x)
    {
      const int8_t v = msg->data[y * msg->info.width + x];
      if (v < 50)
        continue;
      const double wx = ox + (x + 0.5) * res;
      const double wy = oy + (y + 0.5) * res;
      for (double z = 0.15; z <= 1.55; z += 0.35)
        grid.insert(Eigen::Vector3d(wx, wy, z));
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  query_.setStaticGrid(grid);
  have_occ_ = grid.occupiedVoxels() > 0;
}

void VisualQueryAdapter::objectsCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
  // layout: [t_ref, n, (cx,cy,cz,sx,sy,sz,vx,vy,vz) * n]
  const std::vector<double>& d = msg->data;
  if (d.size() < 2)
    return;
  const double t_ref = d[0];
  const int n = static_cast<int>(d[1]);
  std::vector<visual_query::DynamicObject> objs;
  for (int i = 0; i < n; ++i)
  {
    const std::size_t b = 2 + 9 * static_cast<std::size_t>(i);
    if (b + 8 >= d.size())
      break;
    visual_query::DynamicObject o;
    o.center = Eigen::Vector3d(d[b + 0], d[b + 1], d[b + 2]);
    o.size = Eigen::Vector3d(d[b + 3], d[b + 4], d[b + 5]);
    o.vel = Eigen::Vector3d(d[b + 6], d[b + 7], d[b + 8]);
    o.t_ref = t_ref;
    objs.push_back(o);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  objects_.swap(objs);
  objects_t_ref_ = t_ref;
}

void VisualQueryAdapter::refresh()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!have_points_)
  {
    ready_ = false;
    ROS_WARN_STREAM_THROTTLE(5.0, "[visual_query] no map_points yet ("
        << raw_points_.size() << " cached points): visual edges skipped");
    return;
  }

  // Downsample from the *robot's* viewpoint: the snapshot must represent what the
  // camera can actually resolve, not the whole map.
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  if (!raw_points_.empty())
  {
    // crude but stable: the median of the cloud is a good stand-in for "near the robot"
    // when the robot pose is not passed in; callers that know better pass it to
    // setSnapshot themselves.
    origin = raw_points_.front().p;
  }
  std::vector<visual_query::MapPoint> snap =
      visual_query::LocalVisualQuery::downsample(raw_points_, origin,
                                                 static_cast<std::size_t>(params_.max_snapshot), 0.35);
  query_.setSnapshot(snap);
  snapshot_size_ = snap.size();
  query_.setDynamicObjects(objects_);
  ready_ = snap.size() >= static_cast<std::size_t>(params_.min_points);
  if (!ready_)
    ROS_WARN_STREAM_THROTTLE(5.0, "[visual_query] snapshot too small ("
        << snap.size() << " < " << params_.min_points << " points): visual edges skipped");
}

double VisualQueryAdapter::normalizedObservability(double x, double y, double theta, double t) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_)
    return 0.0;
  visual_query::Pose2D p;
  p.x = x;
  p.y = y;
  p.theta = theta;
  p.z = 0.30;                                  // camera height on this platform
  const visual_query::SingleResult r = query_.query(p, t);
  if (r.info_raw <= 0.0)
    return 0.0;
  const double ref = has_ref_ ? ref_info_ : ref_info_;
  // saturating map onto [0,1): more information than the reference saturates smoothly
  const double g = r.info_raw / (r.info_raw + ref);
  return std::min(1.0, std::max(0.0, 2.0 * g));   // 2*g so that "reference" maps to ~1
}

void VisualQueryAdapter::coVisibility(double xa, double ya, double theta_a, double ta,
                                     double xb, double yb, double theta_b, double tb,
                                     double* continuity, double* parallax_min_deg) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready_)
  {
    if (continuity) *continuity = 1.0;
    if (parallax_min_deg) *parallax_min_deg = 180.0;
    return;
  }
  visual_query::Pose2D a, b;
  a.x = xa; a.y = ya; a.theta = theta_a; a.z = 0.30;
  b.x = xb; b.y = yb; b.theta = theta_b; b.z = 0.30;
  const visual_query::PairResult r = query_.query_pair(a, ta, b, tb);
  if (continuity) *continuity = std::min(1.0, std::max(0.0, r.continuity));
  if (parallax_min_deg) *parallax_min_deg = r.parallax_min_deg;
}

} // namespace teb_local_planner

#endif // TEB_HAS_VISUAL_QUERY
