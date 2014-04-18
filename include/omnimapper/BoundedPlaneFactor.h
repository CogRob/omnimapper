#pragma once

#include <gtsam/slam/OrientedPlane3Factor.h>
#include <omnimapper/BoundedPlane3.h>

namespace omnimapper 
{
  template <typename PointT>
  class BoundedPlaneFactor : public gtsam::OrientedPlane3Factor 
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    protected:
      using gtsam::OrientedPlane3Factor::poseKey_;
      using gtsam::OrientedPlane3Factor::landmarkKey_;
      BoundedPlane3<PointT> measured_p_;
      
      typedef NoiseModelFactor2<gtsam::Pose3, BoundedPlane3<PointT> > Base;
      
    public:
      
      BoundedPlaneFactor ()
      {}

      /// Constructor with measured plane coefficients (a,b,c,d), noise model, pose symbol
      BoundedPlaneFactor (const gtsam::Vector&z, CloudPtr boundary, const gtsam::SharedGaussian& noiseModel,
                            gtsam::Key pose,
                            gtsam::Key landmark)
        : gtsam::OrientedPlane3Factor (z, noiseModel, pose, landmark)
      {
        measured_p_ = BoundedPlane3<PointT> (z (0), z (1), z (2), z (3), boundary);
      }

      /// print
      void print(const std::string& s="BoundedPlaneFactor") const;
      
      virtual gtsam::Vector evaluateError(const gtsam::Pose3& pose, const BoundedPlane3<PointT>& plane, 
                                          boost::optional<gtsam::Matrix&> H1 = boost::none,
                                          boost::optional<gtsam::Matrix&> H2 = boost::none) const;
  };
}
