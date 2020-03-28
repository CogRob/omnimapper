/**
 * @file    Plane.h
 * @brief   3D Plane
 * @author  Alex Trevor
 * @author  John Rogers
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Value.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <boost/optional.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
//#include <omnimapper_msgs/PlaneInfos.h>
//#include <omnimapper_msgs/WallFeature.h>
//#include <pcl/PCLHeader.h>
//#include <gtpointcloud/pointcloud_helpers.h>
//#include <omnimapper/math_functions.h>
//#include <mspacegtsam/MathUtils.h>
#include <omnimapper/transform_tools.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/geometry/polygon_operations.h>

// typedef pcl::PointXYZRGBA PointT;
// TODO: find better solution for hull projection. Would be ideal if it were
// auto-updated.

namespace boost {
namespace serialization {

template <class Archive>
void serialize(Archive& ar, pcl::PointXYZ& pt, const unsigned int version) {
  ar& BOOST_SERIALIZATION_NVP(pt.x);
  ar& BOOST_SERIALIZATION_NVP(pt.y);
  ar& BOOST_SERIALIZATION_NVP(pt.z);
}
}  // namespace serialization
}  // namespace boost
namespace gtsam {

/**
 * A 2D point
 * Derived from testable so has standard print and equals, and assert_equals
 * works Functional, so no set functions: once created, a point is constant.
 */
template <typename PointT>
class Plane {
 private:
  // double theta_, phi_,rho_;
  double a_, b_, c_, d_;
  pcl::PointCloud<PointT> hull_;
  pcl::PointCloud<PointT> inliers_;
  bool concave_;
  std::vector<std::vector<float> > out_hull;
  Eigen::Vector4f centroid_;
  // double prev_a_, prev_b_, prev_c_, prev_d_;

 public:
  Plane();

  // Plane(const gtsam::Pose3& pose,
  // const omnimapper_msgs::PlaneInfo& plane_info,
  // const bool& concave=false);
  // Plane(const gtsam::Pose3& pose,
  // const omnimapper_msgs::WallFeature& wall_feature);

  Plane(const gtsam::Pose3& pose, Plane& plane_info,
        const bool& concave = false);

  // //This version makes a plane in the local reference frame, for
  // extend/retract Plane(const omnimapper_msgs::WallFeature& wall_feature);

  Plane(double a, double b, double c, double d,
        const pcl::PointCloud<PointT>& hull,
        const pcl::PointCloud<PointT>& inliers, const bool& concave = false);

  Plane(double a, double b, double c, double d,
        const pcl::PointCloud<PointT>& hull,
        const pcl::PointCloud<PointT>& inliers,
        const Eigen::Vector4f& centroid);

  Plane(double a, double b, double c, double d,
        const pcl::PointCloud<PointT>& hull,
        const pcl::PointCloud<PointT>& inliers);

  PointT MakePoint(float x, float y, float z);

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** equals with an tolerance, prints out message if unequal*/
  bool equals(const Plane& q, double tol = 1e-9) const;

  double a() const { return a_; }
  double b() const { return b_; }
  double c() const { return c_; }
  double d() const { return d_; }
  const pcl::PointCloud<PointT>& hull() const { return hull_; }
  const pcl::PointCloud<PointT>& inliers() const { return inliers_; }

  Matrix GetDh1(const gtsam::Pose3& xr) const;
  Matrix GetDh2(const gtsam::Pose3& xr) const;
  Vector Geth(const Vector& xo, const Vector& measured) const;
  Plane retract(const Vector& d) const;
  Vector localCoordinates(const Plane& p2) const;
  Vector GetXo(const gtsam::Pose3& xr) const;
  void Extend(const Pose3& pose, const gtsam::Plane<PointT>& plane);
  void Extend2(const Pose3& pose, const gtsam::Plane<PointT>& plane);
  void Retract(const Pose3& pose, const gtsam::Plane<PointT>& plane);
  void populateCloud();
  gtsam::Vector GetXf() const;
  // gtsam::Vector GetLinearState(const gtsam::Pose3& xr,
  // 		 const omnimapper_msgs::WallFeature& measured,
  // 		 boost::optional<Matrix&> dhbydxr,
  // 		 boost::optional<Matrix&> dhbydxf) const;

  // Vector GetLinearState(const gtsam::Pose3& xr,
  // 	  const omnimapper_msgs::PlaneInfo& measured,
  // 	  boost::optional<Matrix&> dhbydxr,
  // 	  boost::optional<Matrix&> xhbydxf)const;

  Vector GetLinearState(const gtsam::Pose3& xr, const Plane<PointT>& measured,
                        boost::optional<Matrix&> dhbydxr,
                        boost::optional<Matrix&> xhbydxf) const;

  constexpr static size_t dimension = 4;

  /** return DOF, dimensionality of tangent space */
  size_t dim() const { return Plane<PointT>::dimension; }

  /** return vectorized form (column-wise) */
  Vector vector() const {
    Vector v(4);
    v(0) = a_;
    v(1) = b_;
    v(2) = c_;
    v(3) = d_;
    return v;
  }
  /** operators */
  inline bool operator==(const Plane<PointT>& q) const {
    return a_ == q.a_ && b_ == q.b_ && c_ == q.c_ && d_ == q.d_;
  }

 private:
  /** Serialization function */

  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version) {
    printf("a\n");
    ar& boost::serialization::make_nvp(
        "Plane", boost::serialization::base_object<Value>(*this));
    printf("b\n");
    ar& BOOST_SERIALIZATION_NVP(a_);
    printf("c\n");
    ar& BOOST_SERIALIZATION_NVP(b_);
    printf("d\n");
    ar& BOOST_SERIALIZATION_NVP(c_);
    printf("e\n");
    ar& BOOST_SERIALIZATION_NVP(d_);
    printf("f\n");
    ar& boost::serialization::make_nvp("hull", hull_.points);
    printf("g\n");
  }
};

/** print using member print function, currently used by LieConfig */
// inline void print(const Plane<PointT>& obj, const std::string& str = "") {
//   obj.print(str);
// }
template <typename PointT>
struct traits<Plane<PointT> > : internal::Manifold<Plane<PointT> > {};
}  // namespace gtsam
