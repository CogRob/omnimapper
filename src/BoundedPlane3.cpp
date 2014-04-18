#include <omnimapper/BoundedPlane3.h>

template <typename PointT> void
omnimapper::BoundedPlane3<PointT>::reprojectBoundary ()
{
  
}


template <typename PointT> omnimapper::BoundedPlane3<PointT> 
omnimapper::BoundedPlane3<PointT>::retract (const gtsam::Vector& v) const
{
  // Call the OreintedPlane3 retraction
  OrientedPlane3::retract (v);
  
  // Retract the boundary too.
}
