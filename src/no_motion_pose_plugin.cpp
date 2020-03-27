#include <glog/logging.h>
#include <omnimapper/no_motion_pose_plugin.h>

namespace omnimapper {

NoMotionPosePlugin::NoMotionPosePlugin(omnimapper::OmniMapperBase* mapper)
    : mapper_(mapper) {}

gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr
NoMotionPosePlugin::AddRelativePose(boost::posix_time::ptime t1,
                                    gtsam::Symbol sym1,
                                    boost::posix_time::ptime t2,
                                    gtsam::Symbol sym2) {
  LOG(INFO) << "Adding factor between "
            << std::string(sym1) << " and " << std::string(sym2);
  gtsam::Pose3 relative_pose = gtsam::Pose3::identity();

  // TODO(shengye): Should the following be configurable?
  const double trans_noise = 100.0;
  const double rot_noise = 100.0;

  gtsam::Vector noise_vector(6);
  noise_vector << rot_noise, rot_noise, rot_noise,
                 trans_noise, trans_noise, trans_noise;
  gtsam::SharedDiagonal noise =
      gtsam::noiseModel::Diagonal::Sigmas(noise_vector);
  gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between(
      new gtsam::BetweenFactor<gtsam::Pose3>(sym1, sym2, relative_pose, noise));
  return between;
}

bool NoMotionPosePlugin::Ready() {
  return true;
}

}  // namespace omnimapper
