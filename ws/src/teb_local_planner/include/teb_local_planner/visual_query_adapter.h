/*********************************************************************
 *  EXPLORB-LOCOBS — concrete IVisualQuery backed by the visual_query package.
 *
 *  This is the bridge between ROS data and the pure C++ query module:
 *
 *    /orb_slam2_rgbd/map_points      PointCloud2  -> 80-150 point snapshot
 *                                                    (downsampled once per refresh)
 *    /gridmapper/rectified_map       OccupancyGrid -> static occupancy (2.5D voxels)
 *    /dynamic_objects                Float64MultiArray -> tracked dynamic objects with
 *                                    constant-velocity prediction (measured from depth
 *                                    by dynamic_mask.py, never from simulator truth)
 *
 *  Everything is refreshed by the TEB *outer* loop (refresh()); the inner g2o
 *  iterations only read the resulting field, which is what keeps the optimisation
 *  cheap and keeps the dynamic prediction consistent within one optimisation.
 *********************************************************************/

#ifndef _VISUAL_QUERY_ADAPTER_H_
#define _VISUAL_QUERY_ADAPTER_H_

#ifdef TEB_HAS_VISUAL_QUERY

#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>

#include <teb_local_planner/g2o_types/edge_visual_observability.h>
#include <visual_query/local_visual_query.h>

namespace teb_local_planner
{

class VisualQueryAdapter : public IVisualQuery
{
public:
  VisualQueryAdapter();
  ~VisualQueryAdapter() override;

  /// Subscribe to the three inputs. Safe to call once during plugin initialisation.
  void initialize(ros::NodeHandle& nh, const std::string& map_points_topic,
                  const std::string& occupancy_topic, const std::string& objects_topic);

  /// Rebuild the snapshot / static grid from the latest messages. Called once per
  /// TEB outer planning cycle (so the dynamic prediction is refreshed there).
  void refresh();

  int maskedPointsSeen() const { return static_cast<int>(snapshot_size_); }

  // ---- IVisualQuery -----------------------------------------------------
  double normalizedObservability(double x, double y, double theta, double t) const override;
  void coVisibility(double xa, double ya, double theta_a, double ta,
                    double xb, double yb, double theta_b, double tb,
                    double* continuity, double* parallax_min_deg) const override;
  bool ready() const override { return ready_; }

  /// Reference information used to map the raw query onto [0,1]. Set from the first
  /// few successful queries so that the normalisation does not need a hand-tuned
  /// absolute threshold.
  void setReferenceInformation(double info) { ref_info_ = info; has_ref_ = true; }
  double referenceInformation() const { return ref_info_; }

private:
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr& msg);
  void occupancyCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void objectsCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

  visual_query::LocalVisualQuery query_;
  visual_query::QueryParams params_;

  ros::Subscriber sub_points_, sub_occ_, sub_objs_;
  std::vector<visual_query::MapPoint> raw_points_;
  bool have_points_ = false;
  bool have_occ_ = false;

  std::vector<visual_query::DynamicObject> objects_;
  double objects_t_ref_ = 0.0;

  mutable std::mutex mutex_;
  std::size_t snapshot_size_ = 0;
  bool ready_ = false;

  double ref_info_ = 1.0;
  bool has_ref_ = false;
};

} // namespace teb_local_planner

#endif // TEB_HAS_VISUAL_QUERY
#endif // _VISUAL_QUERY_ADAPTER_H_
