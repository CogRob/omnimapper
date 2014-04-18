#pragma once

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/base/DerivedValue.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace omnimapper 
{
  /**
   * A planar landmark bounded by a polygonal point cloud.
   */
  template <typename PointT>
  class BoundedPlane3 : public gtsam::OrientedPlane3 
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    
    protected:
      using OrientedPlane3::n_;
      using OrientedPlane3::d_;
      
      CloudPtr boundary_;

    public:
      BoundedPlane3 ()
      {}
      
      BoundedPlane3 (double a, double b, double c, double d, CloudPtr boundary) 
        : OrientedPlane3 (a, b, c, d),
          boundary_ (boundary)
      {}
      
      // Override Plane3 retract to additionally reproject the boundary when needed
      BoundedPlane3 retract (const gtsam::Vector& v) const;

      // reprojects the boundary to the current plane coefficients
      void reprojectBoundary ();

      // extend the boundary cloud with a new measurement
      void extendBoundary (const gtsam::Pose3& pose, BoundedPlane3<PointT>& plane);
      
      // retract the boundary cloud to a given measurement
      void retractBoundary (const gtsam::Pose3& pose, BoundedPlane3<PointT>& plane);
  };
  
}
