#pragma once

#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Unit3.h>
#include <omnimapper/OrientedPlane3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/thread/lock_guard.hpp>
#include <boost/thread/mutex.hpp>

namespace omnimapper {
/**
 * A planar landmark bounded by a polygonal point cloud.
 */
template <typename PointT>
class BoundedPlane3 : public gtsam::DerivedValue<BoundedPlane3<PointT> > {
  typedef typename pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef typename boost::shared_ptr<boost::mutex> MutexPtr;
  typedef typename std::pair<CloudPtr, MutexPtr> CloudMutexPtrPair;
  typedef typename std::pair<CloudConstPtr, MutexPtr> CloudConstMutexPtrPair;

 protected:
  gtsam::Unit3 n_;
  double d_;

  boost::shared_ptr<boost::mutex> plane_mutex_;
  CloudPtr boundary_;

 public:
  BoundedPlane3() : plane_mutex_(new boost::mutex()), boundary_(new Cloud()) {}

  BoundedPlane3(const BoundedPlane3<PointT>& plane, bool deep_copy)
      : n_(plane.n_), d_(plane.d_) {
    if (deep_copy) {
      boost::lock_guard<boost::mutex> lock(*(plane.plane_mutex_));
      boundary_ = CloudPtr(new Cloud(*plane.boundary_));
      plane_mutex_ = boost::make_shared<boost::mutex>();
    } else {
      boundary_ = plane.boundary_;
      plane_mutex_ = plane.plane_mutex_;
    }
  }

  // By default we do not perform a deep copy.
  BoundedPlane3(const BoundedPlane3<PointT>& plane)
      : BoundedPlane3(plane, false) {}

  // If no mutex is provided, we perform deep copy, but we assume boundary is
  // already locked.
  BoundedPlane3(const gtsam::Unit3& s, double d, const CloudConstPtr& boundary)
      : n_(s),
        d_(d),
        plane_mutex_(new boost::mutex()),
        boundary_(new Cloud(*boundary)) {}

  BoundedPlane3(const gtsam::Unit3& s, double d,
                const std::pair<CloudPtr, MutexPtr>& cloud_mutex_pair)
      : n_(s),
        d_(d),
        plane_mutex_(cloud_mutex_pair.second),
        boundary_(cloud_mutex_pair.first) {}

  BoundedPlane3(double a, double b, double c, double d,
                const CloudConstPtr& boundary)
      : n_(gtsam::Unit3(gtsam::Point3(a, b, c))),
        d_(d),
        plane_mutex_(new boost::mutex()),
        boundary_(new Cloud(*boundary)) {}

  BoundedPlane3(double a, double b, double c, double d,
                const std::pair<CloudPtr, MutexPtr>& cloud_mutex_pair)
      : n_(gtsam::Unit3(gtsam::Point3(a, b, c))),
        d_(d),
        plane_mutex_(cloud_mutex_pair.second),
        boundary_(cloud_mutex_pair.first) {}

  /// The print fuction
  void print(const std::string& s = std::string()) const;

  /// The equals function with tolerance
  bool equals(const BoundedPlane3<PointT>& s, double tol = 1e-9) const {
    return (n_.equals(s.n_, tol) && (fabs(d_ - s.d_) < tol));
  }

  // Construct from a measurement at some pose, placing the measurement in the
  // map frame
  // BoundedPlane3 (const gtsam::Pose3& pose, BoundedPlane3& plane_measurement);

  /// Computes the error between two poses
  gtsam::Vector error(const BoundedPlane3<PointT>& plane) const;

  /// Dimensionality of tangent space = 3 DOF
  inline static size_t Dim() { return 3; }

  /// Dimensionality of tangent space = 3 DOF
  inline size_t dim() const { return 3; }

  /// Returns the plane coefficients (a, b, c, d)
  gtsam::Vector planeCoefficients() const;

  inline gtsam::Unit3 normal() const { return n_; }

  // Override Plane3 retract to additionally reproject the boundary when needed
  BoundedPlane3 retract(const gtsam::Vector& v) const;

  /// The local coordinates function
  gtsam::Vector localCoordinates(const BoundedPlane3<PointT>& s) const;

  // reprojects the boundary to the current plane coefficients
  void reprojectBoundary();

  // extend the boundary cloud with a new measurement
  void extendBoundary(const gtsam::Pose3& pose,
                      const BoundedPlane3<PointT>& plane) const;

  // retract the boundary cloud to a given measurement
  void retractBoundary(const gtsam::Pose3& pose, BoundedPlane3<PointT>& plane);

  CloudConstMutexPtrPair boundary() const {
    return std::make_pair(static_cast<CloudConstPtr>(boundary_), plane_mutex_);
  }

  CloudMutexPtrPair boundary() {
    return std::make_pair(boundary_, plane_mutex_);
  }

  double d() const { return (d_); }

  static BoundedPlane3 Transform(const omnimapper::BoundedPlane3<PointT>& plane,
                                 const gtsam::Pose3& xr,
                                 boost::optional<gtsam::Matrix&> Hr,
                                 boost::optional<gtsam::Matrix&> Hp);

  static Eigen::Vector4d TransformCoefficients(
      const omnimapper::BoundedPlane3<PointT>& plane, const gtsam::Pose3& xr);
};

}  // namespace omnimapper
