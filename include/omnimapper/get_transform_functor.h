#pragma once

#include <omnimapper/time.h>

#include <Eigen/Geometry>

namespace omnimapper {
/** \brief The GetTransformFunctor enables lookup of a (potentially dynamic)
 * transform at a given time. In particular, this is used to get a transform
 * from a sensor coordinate frame to the base frame. Users can either specify a
 * static transform, or include logic to lookup a dynamic transfrom from the
 * robot's proprioceptive sensing, such as encoders on a robot arm or a pan-tilt
 * unit.
 */
class GetTransformFunctor {
 public:
  virtual Eigen::Affine3d operator()(omnimapper::Time t) = 0;
};

class GetTransformFunctorIdentity : public GetTransformFunctor {
 public:
  Eigen::Affine3d operator()(omnimapper::Time t) {
    return (Eigen::Affine3d::Identity());
  }
};

typedef boost::shared_ptr<omnimapper::GetTransformFunctor>
    GetTransformFunctorPtr;

}  // namespace omnimapper
