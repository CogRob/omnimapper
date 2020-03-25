#pragma once

#include <gtsam/linear/NoiseModel.h>
#include <omnimapper/OrientedPlane3Factor.h>
#include <omnimapper/BoundedPlane3.h>

namespace omnimapper {
template <typename PointT>
class BoundedPlaneFactor
    : public gtsam::NoiseModelFactor2<
          gtsam::Pose3, omnimapper::BoundedPlane3<PointT> >::NoiseModelFactor2 {
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;

 protected:
  // using gtsam::OrientedPlane3Factor::poseKey_;
  // using gtsam::OrientedPlane3Factor::landmarkKey_;
  gtsam::Key poseKey_;
  gtsam::Key landmarkKey_;
  BoundedPlane3<PointT> measured_p_;

  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, BoundedPlane3<PointT> > Base;

 public:
  BoundedPlaneFactor() {}

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose
  /// symbol
  BoundedPlaneFactor(const gtsam::Vector& z, CloudPtr boundary,
                     const gtsam::SharedGaussian& noiseModel, gtsam::Key pose,
                     gtsam::Key landmark)
      : Base(noiseModel, pose, landmark),
        poseKey_(pose),
        landmarkKey_(landmark) {
    measured_p_ = BoundedPlane3<PointT>(z(0), z(1), z(2), z(3), boundary);
  }

  /// print
  void print(const std::string& s = "BoundedPlaneFactor") const;

  virtual gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const BoundedPlane3<PointT>& plane,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const;
};
}  // namespace omnimapper
