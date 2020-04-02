#pragma once

#include <laser_geometry/laser_geometry.h>
#include <ros/ros.h>
//#include <ros/sensor_msgs/laserscan.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <laser_geometry/laser_geometry.h>
#include <omnimapper/ThreadPool.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/canonical_scan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

namespace omnimapper {
/** \brief Canonical Scan Matcher Plugin
 *
 * \author Carlos Nieto
 */
template <typename LScanT>
class CanonicalScanMatcherPlugin {
  typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> >
      BetweenPose3Ptr;
  typedef typename boost::shared_ptr<sensor_msgs::LaserScan> LaserScanPtr;
  typedef const typename boost::shared_ptr<sensor_msgs::LaserScan>
      LaserScanPConstPtr;
  typedef typename sensor_msgs::LaserScan LScan;

 public:
  CanonicalScanMatcherPlugin(omnimapper::OmniMapperBase* mapper);
  ~CanonicalScanMatcherPlugin();
  void LaserScanCallback(const LaserScanPtr& lscan);
  void Spin();
  bool SpinOnce();
  bool GetBaseToLaserTf(const std::string& frame_id);
  bool AddConstraint(gtsam::Symbol sym1, gtsam::Symbol sym2,
                     scan_tools::CanonicalScan& cscan, bool always_add = true);
  bool TryLoopClosure(gtsam::Symbol sym);
  bool Ready();
  void SetTriggeredMode(bool triggered_mode) {
    triggered_mode_ = triggered_mode;
  }
  void Trigger() { triggered_ = true; }
  void SetDebug(bool debug) { debug_ = debug; }
  LaserScanPConstPtr GetLaserScanPtr(gtsam::Symbol sym);
  sensor_msgs::PointCloud2 GetPC2(gtsam::Symbol sym);

 protected:
  ros::NodeHandle nh_;
  OmniMapperBase* mapper_;
  tf::TransformListener tf_listener_;
  laser_geometry::LaserProjection projector_;
  scan_tools::CanonicalScan seq_canonical_scan_;
  scan_tools::CanonicalScan lc_canonical_scan_;
  bool initialized_;
  std::map<gtsam::Symbol, LaserScanPtr> lscans_;
  std::map<gtsam::Symbol, gtsam::Point3> lscan_centroids_;
  std::map<gtsam::Symbol, sensor_msgs::PointCloud2> clouds_;
  LaserScanPtr current_lscan_;
  bool have_new_lscan_;
  boost::mutex current_cloud_mutex_;
  boost::mutex current_lscan_mutex_;
  bool first_;
  double trans_noise_;
  double rot_noise_;
  bool debug_;
  bool overwrite_timestamps_;
  float leaf_size_;
  float score_threshold_;
  bool downsample_;
  std::string base_frame_name_;
  gtsam::Symbol previous_sym_;
  gtsam::Symbol previous2_sym_;
  gtsam::Symbol previous3_sym_;
  bool add_identity_on_failure_;
  bool add_multiple_links_;
  bool add_loop_closures_;
  bool paused_;
  int count_;
  bool triggered_mode_;
  bool triggered_;
  ros::Time triggered_time_;
  tf::Transform base_to_laser_;
  tf::Transform laser_to_base_;
  ros::Publisher visualization_marker_array_pub_;
  ros::Publisher laser_scan_msg_pub1_;
  ros::Publisher laser_scan_msg_pub2_;
  ThreadPool thread_pool_;
};

}  // namespace omnimapper

gtsam::Pose3 doCSM_impl(const sensor_msgs::LaserScan& from_scan,
                        const sensor_msgs::LaserScan& to_scan,
                        const gtsam::Pose3& odometry_relative_pose,
                        bool& worked,
                        gtsam::noiseModel::Gaussian::shared_ptr& noise_model,
                        scan_tools::CanonicalScan& canonicalScan,
                        // const tf::StampedTransform& base_to_laser_tf,
                        tf::Transform& base_to_laser_tf, bool laser_mode = true,
                        bool debug = false);

sensor_msgs::LaserScan SmoothScan(const sensor_msgs::LaserScanConstPtr& msg_in);

gtsam::Point3 GetMeanLaserPoint(
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud);
