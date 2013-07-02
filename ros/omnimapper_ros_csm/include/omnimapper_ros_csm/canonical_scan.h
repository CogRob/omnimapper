#pragma once

#include <ros/ros.h>
#include <omnimapper_ros_csm/math_functions.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
namespace scan_tools {

  class CanonicalScan {
  protected:
    sm_params input_;
    sm_result output_;
    double cloud_range_min_;
    double cloud_range_max_;
    bool use_cloud_input_;
  public:    
    CanonicalScan();
    void initParams(ros::NodeHandle& nh_private_);
    bool processScan(LDP& curr_ldp_scan, LDP& prev_ldp_scan,
                     const gtsam::Pose2& initial_rel_pose,
                     gtsam::Pose2& output_rel_pose,
		     gtsam::noiseModel::Gaussian::shared_ptr& icp_cov);
    void PointCloudToLDP(const sensor_msgs::PointCloud& cloud,
			 const sensor_msgs::LaserScan& scan,
			 LDP& ldp);
    void laserScanToLDP(const sensor_msgs::LaserScan& scan_msg,
			LDP& ldp);
  };

}
