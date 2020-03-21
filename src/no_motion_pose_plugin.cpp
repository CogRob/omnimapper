#include <omnimapper/no_motion_pose_plugin.h>

namespace
omnimapper
{

  NoMotionPosePlugin::NoMotionPosePlugin (omnimapper::OmniMapperBase* mapper)
    : mapper_ (mapper)
  {

  }

  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr
  NoMotionPosePlugin::addRelativePose (boost::posix_time::ptime t1, gtsam::Symbol sym1, boost::posix_time::ptime t2, gtsam::Symbol sym2)
  {
    printf ("NoMotionPosePlugin: Adding factor between %d and %d\n", sym1.index (), sym2.index ());
    //Eigen::Matrix4f cloud_tform = Eigen::Matrix4f::Identity ();
    gtsam::Pose3 relative_pose = gtsam::Pose3::identity();//(gtsam::Rot3 (cloud_tform.block (0, 0, 3, 3).cast<double>()),
      //gtsam::Point3 (cloud_tform (0,3), cloud_tform (1,3), cloud_tform (2,3)));
    double trans_noise = 100.0;//0.1;//100.0;
    double rot_noise = 100.0;//0.1;//100.0;
    gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas ((gtsam::Vector (6) << rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise).finished());
    //omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym2, sym1, relative_pose, noise));
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between (new gtsam::BetweenFactor<gtsam::Pose3> (sym1, sym2, relative_pose, noise));
    //mapper_->addFactor (between);
    return (between);
  }

  bool
  NoMotionPosePlugin::ready ()
  {
    return (true);
  }

}
