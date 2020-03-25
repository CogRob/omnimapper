/*
 * @file OrientedPlane3Factor.cpp
 * @brief OrientedPlane3 Factor class
 * @author Alex Trevor
 * @date December 22, 2013
 */

#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <omnimapper/OrientedPlane3.h>

namespace omnimapper {

/**
 * Factor to measure a planar landmark from a given pose
 */
class OrientedPlane3Factor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, OrientedPlane3> {
 protected:
  gtsam::Key poseKey_;
  gtsam::Key landmarkKey_;
  gtsam::Vector measured_coeffs_;
  OrientedPlane3 measured_p_;

  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, OrientedPlane3> Base;

 public:
  /// Constructor
  OrientedPlane3Factor() {}
  virtual ~OrientedPlane3Factor() {}

  /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose
  /// symbol
  OrientedPlane3Factor(const gtsam::Vector& z,
                       const gtsam::SharedGaussian& noiseModel,
                       const gtsam::Key& pose, const gtsam::Key& landmark)
      : Base(noiseModel, pose, landmark),
        poseKey_(pose),
        landmarkKey_(landmark),
        measured_coeffs_(z) {
    measured_p_ = OrientedPlane3(gtsam::Unit3(z(0), z(1), z(2)), z(3));
  }

  /// print
  virtual void print(const std::string& s = "OrientedPlane3Factor",
                     const gtsam::KeyFormatter& keyFormatter =
                         gtsam::DefaultKeyFormatter) const;

  /// evaluateError
  virtual gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const OrientedPlane3& plane,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    OrientedPlane3 predicted_plane =
        OrientedPlane3::Transform(plane, pose, H1, H2);
    gtsam::Vector err(3);
    err << predicted_plane.error(measured_p_);
    return (err);
  };
};

// TODO: Convert this factor to dimension two, three dimensions is redundant for
// direction prior
class OrientedPlane3DirectionPrior
    : public gtsam::NoiseModelFactor1<OrientedPlane3> {
 protected:
  OrientedPlane3 measured_p_;  /// measured plane parameters
  gtsam::Key landmarkKey_;
  typedef NoiseModelFactor1<OrientedPlane3> Base;

 public:
  typedef OrientedPlane3DirectionPrior This;
  /// Constructor
  OrientedPlane3DirectionPrior() {}

  /// Constructor with measured plane coefficients (a,b,c,d), noise model,
  /// landmark symbol
  OrientedPlane3DirectionPrior(gtsam::Key key, const gtsam::Vector& z,
                               const gtsam::SharedGaussian& noiseModel)
      : Base(noiseModel, key), landmarkKey_(key) {
    measured_p_ = OrientedPlane3(gtsam::Unit3(z(0), z(1), z(2)), z(3));
  }

  /// print
  virtual void print(const std::string& s = "OrientedPlane3DirectionPrior",
                     const gtsam::KeyFormatter& keyFormatter =
                         gtsam::DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

  virtual gtsam::Vector evaluateError(
      const OrientedPlane3& plane,
      boost::optional<gtsam::Matrix&> H = boost::none) const;
};

}  // namespace omnimapper
