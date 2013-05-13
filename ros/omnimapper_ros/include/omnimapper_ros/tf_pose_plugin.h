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

      bool initialized_;
      
    public:
      TFPosePlugin (omnimapper::OmniMapperBase* mapper);

      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr addRelativePose (boost::posix_time::ptime t1, gtsam::Symbol sym1, boost::posix_time::ptime t2, gtsam::Symbol sym2);
      bool ready ();
  };
}
