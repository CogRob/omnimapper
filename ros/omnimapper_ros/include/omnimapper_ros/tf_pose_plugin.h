#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <omnimapper/pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <omnimapper_ros/ros_time_utils.h>
#include <omnimapper_ros/ros_tf_utils.h>

namespace omnimapper
{
  // Forward Declaration
  class OmniMapperBase;

  /** \brief TFPosePlugin is used to add constraints between consecutive poses in a SLAM problem, such as odometry. */
  class TFPosePlugin : public omnimapper::PosePlugin
  {
    protected:
      /** \brief A reference to the mapper we're a plugin for. */
      OmniMapperBase* mapper_;

      /** \brief A nodehandle. */
      ros::NodeHandle nh_;
      
      /** \brief A TF listener. */
      tf::TransformListener tf_listener_;

      std::string odom_frame_name_;
      
      std::string base_frame_name_;

      double rotation_noise_;
      double roll_noise_, pitch_noise_, yaw_noise_;
      
      double translation_noise_;

      bool initialized_;

      bool debug_;
      
    public:
      TFPosePlugin (omnimapper::OmniMapperBase* mapper);

      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr addRelativePose (boost::posix_time::ptime t1, gtsam::Symbol sym1, boost::posix_time::ptime t2, gtsam::Symbol sym2);
      bool ready ();
      void setOdomFrameName (std::string& odom_frame_name) { odom_frame_name_ = odom_frame_name; }
      void setBaseFrameName (std::string& base_frame_name) { base_frame_name_ = base_frame_name; }
      void setRotationNoise (double rotation_noise) {rotation_noise_ = rotation_noise; roll_noise_ = rotation_noise; pitch_noise_ = rotation_noise; yaw_noise_ = rotation_noise; }
      void setRollNoise (double roll_noise) {roll_noise_ = roll_noise; }
      void setPitchNoise (double pitch_noise) {pitch_noise_ = pitch_noise; }
      void setYawNoise (double yaw_noise) {yaw_noise_ = yaw_noise; }
      void setTranslationNoise (double translation_noise) {translation_noise_ = translation_noise; }
  };
}
