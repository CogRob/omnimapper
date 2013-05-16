#include <omnimapper_ros/tf_pose_plugin.h>

namespace
omnimapper
{

  TFPosePlugin::TFPosePlugin (omnimapper::OmniMapperBase* mapper) 
    : mapper_ (mapper),
      nh_ ("~"),
      tf_listener_ (ros::Duration (30.0))
  {
    
  }

  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr
  TFPosePlugin::addRelativePose (boost::posix_time::ptime t1, gtsam::Symbol sym1, boost::posix_time::ptime t2, gtsam::Symbol sym2)
  {
    // Convert the timestamps
    ros::Time rt1 = ptime2rostime (t1);
    ros::Time rt2 = ptime2rostime (t2);

    // Get the poses
    tf::StampedTransform tf1;
    tf::StampedTransform tf2;

    // Add the factor
    try{
      tf_listener_.waitForTransform("/odom", "/camera_depth_optical_frame",
                                    rt1, ros::Duration(5.0));
      
      tf_listener_.lookupTransform("/odom" , "/camera_depth_optical_frame",
                                   rt1, tf1);

      tf_listener_.waitForTransform("/odom", "/camera_depth_optical_frame",
                                    rt2, ros::Duration(5.0));
      
      tf_listener_.lookupTransform("/odom" , "/camera_depth_optical_frame",
                                   rt2, tf2);
    } catch(tf::TransformException ex) {
      ROS_INFO("OMNImapper reports :: Transform from %s to %s not yet available.  Exception: %s", "/odom", "/base_link",ex.what());
    }

    gtsam::Pose3 pose1 = tf2pose3 (tf1);
    gtsam::Pose3 pose2 = tf2pose3 (tf2);
    gtsam::Pose3 relative_pose = pose2.between (pose1);

    printf ("TFPosePlugin: Adding factor between %d and %d\n", sym1.index (), sym2.index ());
    printf ("TFPosePlugin: Relative transform: %lf %lf %lf\n", relative_pose.x (), relative_pose.y (), relative_pose.z ());
    
    double trans_noise = 1.0;
    double rot_noise = 1.0;
    gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));
    //omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym2, sym1, relative_pose, noise));
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym2, sym1, relative_pose, noise));
    //mapper_->addFactor (between);
    return (between);
  }

  bool
  TFPosePlugin::ready ()
  {
    return (true);
  }

}
