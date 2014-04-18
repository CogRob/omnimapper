#include <omnimapper/BoundedPlaneFactor.h>

namespace omnimapper
{
//***************************************************************************
  template <typename PointT>
  gtsam::Vector BoundedPlaneFactor<PointT>::evaluateError(const gtsam::Pose3& pose, const BoundedPlane3<PointT>& plane, 
                                                          boost::optional<gtsam::Matrix&> H1,
                                                          boost::optional<gtsam::Matrix&> H2) const 
  {
    BoundedPlane3<PointT> predicted_plane = BoundedPlane3<PointT>::Transform (plane, pose, H1, H2);
    gtsam::Vector error = predicted_plane.error (measured_p_);
    return (error);
  }
}
