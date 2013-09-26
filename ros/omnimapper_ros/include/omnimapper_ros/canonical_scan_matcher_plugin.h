#pragma once

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
//#include <ros/sensor_msgs/laserscan.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/canonical_scan.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Symbol.h>

namespace omnimapper
{
  /** \brief Canonical Scan Matcher Plugin
   *
   * \author Carlos Nieto
   */
  template <typename LScanT>
  class CanonicalScanMatcherPlugin
  {
    typedef typename boost::shared_ptr<gtsam::BetweenFactor<gtsam::Pose3> > BetweenPose3Ptr;
    typedef typename boost::shared_ptr<sensor_msgs::LaserScan> LaserScanPtr;
    typedef const typename boost::shared_ptr<sensor_msgs::LaserScan> LaserScanPConstPtr;
    typedef typename sensor_msgs::LaserScan LScan;

    public:

      CanonicalScanMatcherPlugin (omnimapper::OmniMapperBase* mapper);
      ~CanonicalScanMatcherPlugin ();
      void laserScanCallback (const LaserScanPtr& lscan);
      void spin();
      bool spinOnce();
      bool getBaseToLaserTf (const std::string& frame_id);
      bool addConstraint (gtsam::Symbol sym1, gtsam::Symbol sym2, bool always_add=true);
      bool tryLoopClosure (gtsam::Symbol sym);
      bool ready ();
      void setTriggeredMode (bool triggered_mode) { triggered_mode_ = triggered_mode; }
      void trigger () { triggered_ = true; }
      LaserScanPConstPtr getLaserScanPtr (gtsam::Symbol sym);
      sensor_msgs::PointCloud2 getPC2 (gtsam::Symbol sym);


    protected:
      ros::NodeHandle nh_;
      OmniMapperBase* mapper_;
      tf::TransformListener tf_listener_;
      laser_geometry::LaserProjection projector_;
      scan_tools::CanonicalScan canonical_scan_;
      bool initialized_;
      std::map<gtsam::Symbol, LaserScanPtr> lscans_;
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
  };








}

gtsam::Pose3 doCSM_impl ( const sensor_msgs::LaserScan& from_scan,
                          const sensor_msgs::LaserScan& to_scan,
                          const gtsam::Pose3& odometry_relative_pose,
                          bool& worked,
                          gtsam::noiseModel::Gaussian::shared_ptr& noise_model,
                          scan_tools::CanonicalScan& canonicalScan,
                          //const tf::StampedTransform& base_to_laser_tf,
                          tf::Transform& base_to_laser_tf,
                          bool laser_mode = true);

sensor_msgs::LaserScan SmoothScan (const sensor_msgs::LaserScanConstPtr& msg_in);

gtsam::Point3 GetMeanLaserPoint(const sensor_msgs::PointCloud& cloud);

