/**
 * @file    Plane.cpp
 * @brief   3D Plane
 * @author  Alex Trevor
 * @author  John Rogers
 */

#include <omnimapper/plane.h>
#include <omnimapper/transform_tools.h>
//#include <gtpointcloud/pointcloud_helpers.h>
//#include <pcl_ros/transforms.h>
#include <pcl/io/io.h>
//#include <tf/transform_broadcaster.h>
#include <omnimapper/geometry.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <omnimapper/impl/geometry.hpp>
#include <pcl/geometry/impl/polygon_operations.hpp>

#define KILL_INLIERS 1

using namespace std;

template class gtsam::Plane<pcl::PointXYZRGBA>;

namespace gtsam {

/* ************************************************************************* */
template <typename PointT>
Plane<PointT>::Plane() : a_(0), b_(0), c_(0), d_(0) {
  // printf("Warning: Using default constructor in plane.  This might indicate a
  // bug\n"); assert(false); This might not be a bug, because the serialization
  // interface uses the default constructor and then loads the data after
  // construction.

  concave_ = false;
}

/*
Plane::Plane(const gtsam::Pose3& pose, const omnimapper_msgs::PlaneInfo&
plane_info, const bool& concave)
{
  tf::Transform pose2map = Pose3ToTransform(pose);
  //tf::Transform map2pose = pose2map.inverse();
  pcl::PointCloud<PointT> meas_hull;
  pcl::fromROSMsg(plane_info.hull,meas_hull);
  pcl::PointCloud<PointT> meas_inliers;
  pcl::fromROSMsg(plane_info.cloud,meas_inliers);
  pcl_ros::transformPointCloud(meas_hull,hull_,pose2map);
  pcl_ros::transformPointCloud(meas_inliers,inliers_,pose2map);

  Eigen::Vector4f map_normal;
  gtsam::Point3
normal_in(plane_info.model.values[0],plane_info.model.values[1],plane_info.model.values[2]);
  gtsam::Point3 normal_out = pose.rotation().rotate(normal_in);
  a_ = normal_out.x();
  b_ = normal_out.y();
  c_ = normal_out.z();
  d_ = -1 * (hull_.points[0].x * a_ + hull_.points[0].y * b_ + hull_.points[0].z
* c_);
}
*/

//   Plane::Plane(const gtsam::Pose3& pose, const omnimapper_msgs::PlaneInfo&
//   plane_info, const bool& concave)
//   {
//     tf::Transform pose2map = Pose3ToTransform(pose);
//     //tf::Transform map2pose = pose2map.inverse();
//     pcl::PointCloud<PointT> meas_hull;
//     pcl::fromROSMsg(plane_info.hull,meas_hull);
//     pcl::PointCloud<PointT> meas_inliers;
//     pcl::fromROSMsg(plane_info.cloud,meas_inliers);
//     pcl_ros::transformPointCloud(meas_hull,hull_,pose2map);
// #ifndef KILL_INLIERS
//     pcl_ros::transformPointCloud(meas_inliers,inliers_,pose2map);
// #endif
//     Eigen::Vector4f map_normal;
//     gtsam::Point3
//     normal_in(plane_info.model.values[0],plane_info.model.values[1],plane_info.model.values[2]);
//     gtsam::Point3 normal_out = pose.rotation().rotate(normal_in);

//     gtsam::Pose3 pose_inv = pose.inverse();

//     a_ = normal_out.x();
//     b_ = normal_out.y();
//     c_ = normal_out.z();
//     d_ =
//       plane_info.model.values[0] * pose_inv.x() +
//       plane_info.model.values[1] * pose_inv.y() +
//       plane_info.model.values[2] * pose_inv.z() +
//       plane_info.model.values[3];
//     concave_ = concave;

//   }
template <typename PointT>
PointT Plane<PointT>::MakePoint(float x, float y, float z) {
  PointT pt;
  pt.x = x;
  pt.y = y;
  pt.z = z;
  return pt;
}
//   Plane::Plane(const gtsam::Pose3& pose,
// 	       const omnimapper_msgs::WallFeature& wf) {
//     concave_ = false;
//     tf::Transform pose2map = Pose3ToTransform(pose);
//     pcl::PointCloud<PointT> meas_hull;
//     meas_hull.push_back(MakePoint(wf.p1.x, wf.p1.y, wf.p1.z + 0.05f));
//     meas_hull.push_back(MakePoint(wf.p1.x, wf.p1.y, wf.p1.z - 0.05f));
//     meas_hull.push_back(MakePoint(wf.p2.x, wf.p2.y, wf.p2.z - 0.05f));
//     meas_hull.push_back(MakePoint(wf.p2.x, wf.p2.y, wf.p2.z + 0.05f));
//     pcl_ros::transformPointCloud(meas_hull,hull_,pose2map);
// #ifndef KILL_INLIERS
//     pcl_ros::transformPointCloud(meas_hull,inliers_,pose2map);
// #endif

//     gtsam::Vector p1 = gtsam::Vector_(3, wf.p1.x, wf.p1.y, wf.p1.z);
//     gtsam::Vector p2 = gtsam::Vector_(3, wf.p2.x, wf.p2.y, wf.p2.z);
//     gtsam::Vector b = (p2 - p1)/((p2-p1).norm());
//     gtsam::Point3 normal_in(-b(1), b(0), 0.0);

//     Eigen::Vector4f map_normal;

//     gtsam::Point3 normal_out = pose.rotation().rotate(normal_in);
//     gtsam::Pose3 pose_inv = pose.inverse();

//     double dx = p2.x() - p1.x();
//     double dy = p2.y() - p1.y();

//     double range = (dy*p1.x() - dx*p1.y())/sqrt(dx*dx + dy*dy);
//     a_ = normal_out.x();
//     b_ = normal_out.y();
//     c_ = normal_out.z();
//     d_ =
//       normal_in.x() * pose_inv.x() +
//       normal_in.y() * pose_inv.y() +
//       normal_in.z() * pose_inv.z() +
//       range;
//   }
// Make a plane in the local reference frame for extend/retract
// Plane::Plane(const omnimapper_msgs::WallFeature& wf) {
//   concave_ = false;
//   hull_.push_back(MakePoint(wf.p1.x, wf.p1.y, wf.p1.z + 0.05f));
//   hull_.push_back(MakePoint(wf.p1.x, wf.p1.y, wf.p1.z - 0.05f));
//   hull_.push_back(MakePoint(wf.p2.x, wf.p2.y, wf.p2.z - 0.05f));
//   hull_.push_back(MakePoint(wf.p2.x, wf.p2.y, wf.p2.z + 0.05f));
//   gtsam::Vector p1 = gtsam::Vector_(3, wf.p1.x, wf.p1.y, wf.p1.z);
//   gtsam::Vector p2 = gtsam::Vector_(3, wf.p2.x, wf.p2.y, wf.p2.z);
//   gtsam::Vector b = (p2 - p1)/((p2-p1).norm());
//   gtsam::Point3 normal_in(-b(1), b(0), 0.0);

//   double dx = p2.x() - p1.x();
//   double dy = p2.y() - p1.y();

//   double range = (dy*p1.x() - dx*p1.y())/sqrt(dx*dx + dy*dy);
//   a_ = normal_in.x();
//   b_ = normal_in.y();
//   c_ = normal_in.z();
//   d_ = range;
// }

template <typename PointT>
Plane<PointT>::Plane(double a, double b, double c, double d,
                     const pcl::PointCloud<PointT>& hull,
                     const pcl::PointCloud<PointT>& inliers,
                     const bool& concave)
    : a_(a),
      b_(b),
      c_(c),
      d_(d),
      hull_(hull),
#ifndef KILL_INLIERS

      inliers_(inliers),
#endif
      concave_(concave) {
}

template <typename PointT>
Plane<PointT>::Plane(const gtsam::Pose3& pose, Plane& plane_info,
                     const bool& concave) {
  Eigen::Affine3f pose2map = pose3ToTransform(pose);
  pcl::PointCloud<PointT> meas_hull = plane_info.hull();
  pcl::transformPointCloud(meas_hull, hull_, pose2map);

  Eigen::Vector4f map_normal;
  // gtsam::Point3
  // normal_in(plane_info.model.values[0],plane_info.model.values[1],plane_info.model.values[2]);
  gtsam::Point3 normal_in(plane_info.a(), plane_info.b(), plane_info.c());
  gtsam::Point3 normal_out = pose.rotation().rotate(normal_in);

  gtsam::Pose3 pose_inv = pose.inverse();

  a_ = normal_out.x();
  b_ = normal_out.y();
  c_ = normal_out.z();
  //    d_ =
  //  plane_info.model.values[0] * pose_inv.x() +
  //  plane_info.model.values[1] * pose_inv.y() +
  //  plane_info.model.values[2] * pose_inv.z() +
  //  plane_info.model.values[3];
  d_ = plane_info.a() * pose_inv.x() + plane_info.b() * pose_inv.y() +
       plane_info.c() * pose_inv.z() + plane_info.d();
  concave_ = concave;
}

template <typename PointT>
Plane<PointT>::Plane(double a, double b, double c, double d,
                     const pcl::PointCloud<PointT>& hull,
                     const pcl::PointCloud<PointT>& inliers,
                     const Eigen::Vector4f& centroid)
    : a_(a),
      b_(b),
      c_(c),
      d_(d),
      hull_(hull),
      inliers_(inliers),
      centroid_(centroid) {
  concave_ = false;
}

template <typename PointT>
Plane<PointT>::Plane(double a, double b, double c, double d,
                     const pcl::PointCloud<PointT>& hull,
                     const pcl::PointCloud<PointT>& inliers)
    : a_(a), b_(b), c_(c), d_(d), hull_(hull), inliers_(inliers) {
  concave_ = false;
}

// Plane::Plane(double a, double b,
//        double c, double d,
//        const pcl::PointCloud<PointT>& hull,
//        const pcl::PointCloud<PointT>& inliers,
//        const bool& concave)
//   : a_(a), b_(b), c_(c), d_(d),
//     hull_(hull),
//     inliers_(inliers),
//     concave_(concave)
// {
// }

template <typename PointT>
void Plane<PointT>::print(const string& s) const {
  cout << s << "(" << a_ << ", " << b_ << ", " << c_ << ", " << d_ << ")"
       << endl;
}

/* ************************************************************************* */
template <typename PointT>
bool Plane<PointT>::equals(const Plane& q, double tol) const {
  return (fabs(a_ - q.a()) < tol && fabs(b_ - q.b()) < tol &&
          fabs(c_ - q.c()) < tol && fabs(d_ - q.d()) < tol);
}

template <typename PointT>
Plane<PointT> Plane<PointT>::retract(const Vector& d) const {
  if (d.size() == 0) {
    return *this;
  }

  Eigen::Vector3d new_norm(a_ + d(0), b_ + d(1), c_ + d(2));
  new_norm.normalize();

  std::cout << "v: " << d << std::endl;
  std::cout << "retracting: " << a_ << " " << b_ << " " << c_ << " " << d_
            << std::endl;
  std::cout << "retracted:  " << new_norm(0) << " " << new_norm(1) << " "
            << new_norm(2) << "d" << std::endl;

  bool update_hull_from_projection = false;
  pcl::PointCloud<PointT> map_hull_on_map;
  if (update_hull_from_projection) {
    pcl::ModelCoefficients map_model;
    map_model.values.push_back(new_norm[0]);
    map_model.values.push_back(new_norm[1]);
    map_model.values.push_back(new_norm[2]);
    map_model.values.push_back(d_ + d(3));
    // pcl::PointCloud<PointT> map_hull_on_map;
    pcl::ProjectInliers<PointT> proj1;
    proj1.setModelType(pcl::SACMODEL_PLANE);
    proj1.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(hull_));
    proj1.setModelCoefficients(
        boost::make_shared<pcl::ModelCoefficients>(map_model));
    proj1.filter(map_hull_on_map);
  } else {
    Eigen::Vector3d prev_norm(a_, b_, c_);
    double angle = acos(prev_norm.dot(new_norm));
    Eigen::Vector3d axis = prev_norm.cross(new_norm);
    axis.normalize();
    Eigen::Affine3d transform;
    // If the normal is near identical, we'll get an invalid transform, and
    // should instead use identity
    if ((pcl_isfinite(angle)) && (pcl_isfinite(axis[0])) &&
        (pcl_isfinite(axis[1])) && (pcl_isfinite(axis[2]))) {
      transform = Eigen::AngleAxisd(angle, axis);
    } else {
      transform = Eigen::Affine3d::Identity();
    }
    pcl::PointCloud<PointT> temp;
    Eigen::Vector3d translation_part = new_norm * -1 * d(3);
    transform.translation() = translation_part;
    pcl::transformPointCloud(hull_, temp, transform);

    std::cout << "plane::retract: angle: " << angle << " axis: " << axis
              << " trans: " << translation_part << std::endl;

    bool project = false;
    if (project) {
      pcl::ModelCoefficients map_model;
      map_model.values.push_back(new_norm[0]);
      map_model.values.push_back(new_norm[1]);
      map_model.values.push_back(new_norm[2]);
      map_model.values.push_back(d_ + d(3));
      // pcl::PointCloud<PointT> map_hull_on_map;
      pcl::ProjectInliers<PointT> proj1;
      proj1.setModelType(pcl::SACMODEL_PLANE);
      proj1.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(temp));
      proj1.setModelCoefficients(
          boost::make_shared<pcl::ModelCoefficients>(map_model));
      proj1.filter(map_hull_on_map);
    } else {
      map_hull_on_map = temp;
    }

    bool verify_ptp_dist = true;
    if (verify_ptp_dist) {
      double orig_dist = 0.0;
      double new_dist = 0.0;
      for (int i = 0; i < hull_.points.size(); i++) {
        double ptp_dist = fabs(new_norm[0] * hull_.points[i].x +
                               new_norm[1] * hull_.points[i].y +
                               new_norm[2] * hull_.points[i].z + (d_ + d(3)));
        double ptp_dist_new =
            fabs(new_norm[0] * map_hull_on_map.points[i].x +
                 new_norm[1] * map_hull_on_map.points[i].y +
                 new_norm[2] * map_hull_on_map.points[i].z + (d_ + d(3)));
        orig_dist += ptp_dist;
        new_dist += ptp_dist_new;
        // if (ptp_dist >= 0.001)
        // {
        //   printf ("Error in Planeretract!  PTP dist not zero after update:
        //   %lf\n", ptp_dist); printf ("Point: %lf %lf %lf\n",
        //   hull_.points[i].x, hull_.points[i].y, hull_.points[i].z);
        // }
        if (!pcl::isFinite(map_hull_on_map.points[i]))
          printf("plane::retract: error -- nan point!\n");
      }
      orig_dist = orig_dist / hull_.points.size();
      new_dist = new_dist / map_hull_on_map.points.size();
      std::cout << "plane::retract: orig dist: " << orig_dist
                << " new_dist: " << new_dist << std::endl;
    }
  }

  Plane new_p(new_norm[0], new_norm[1], new_norm[2], d_ + d(3), map_hull_on_map,
              inliers_, concave_);

  return new_p;
}

template <typename PointT>
Vector Plane<PointT>::localCoordinates(const Plane& p2) const {
  assert(false);
  return gtsam::zero(3);
}

template <typename PointT>
Vector Plane<PointT>::GetXo(const gtsam::Pose3& xr) const {
  Eigen::Vector4f pred;
  gtsam::Point3 normal_in(a_, b_, c_);
  gtsam::Point3 normal_out = xr.rotation().unrotate(normal_in);
  pred[0] = normal_out.x();
  pred[1] = normal_out.y();
  pred[2] = normal_out.z();
  pred[3] = a_ * xr.x() + b_ * xr.y() + c_ * xr.z() + d_;
  gtsam::Vector g_v(4);
  g_v << pred[0], pred[1], pred[2], pred[3];
  return g_v;
}

template <typename PointT>
gtsam::Matrix Plane<PointT>::GetDh1(const gtsam::Pose3& xr) const {
  Matrix Dh1 = gtsam::zeros(4, 6);
  Matrix H1;
  gtsam::Point3 normal_in(a_, b_, c_);
  gtsam::Point3 normal_out = xr.rotation().unrotate(normal_in, H1);
  // Eigen::RowVector3d n(a_,b_,c_);
  // Eigen::RowVector3d p = n * xr.rotation().matrix();
  Eigen::Vector3d n(a_, b_, c_);
  Eigen::Vector3d p = xr.rotation().transpose() * n;
  insertSub(Dh1, H1, 0, 0);
  Dh1(3, 0) = 0;
  Dh1(3, 1) = 0;
  Dh1(3, 2) = 0;
  Dh1(3, 3) = p(0);  // a_;
  Dh1(3, 4) = p(1);  // b_;
  Dh1(3, 5) = p(2);  // c_;

  return Dh1;
}

template <typename PointT>
gtsam::Matrix Plane<PointT>::GetDh2(const gtsam::Pose3& xr) const {
  Matrix Dh2 = gtsam::zeros(4, 4);
  Matrix H1;
  Matrix H2;
  gtsam::Point3 normal_in(a_, b_, c_);
  gtsam::Point3 normal_out = xr.rotation().unrotate(normal_in, H1, H2);
  //    Eigen::Vector3d n(xr.x(),xr.y(),xr.z());
  //    Eigen::Vector3d p = xr.rotation().transpose() * n;
  insertSub(Dh2, H2, 0, 0);

  Dh2(0, 3) = 0;
  Dh2(1, 3) = 0;
  Dh2(2, 3) = 0;

  Dh2(3, 0) = xr.x();
  Dh2(3, 1) = xr.y();
  Dh2(3, 2) = xr.z();
  Dh2(3, 3) = 1;

  return Dh2;
}

template <typename PointT>
gtsam::Vector Plane<PointT>::Geth(const Vector& xo,
                                  const Vector& measured) const {
  // printf("geth: xo: %lf %lf %lf %lf\n",xo(0),xo(1),xo(2),xo(3));
  // printf("geth: ms: %lf %lf %lf
  // %lf\n",measured(0),measured(1),measured(2),measured(3));
  Vector h = gtsam::zero(4);
  h[0] = xo(0) - measured(0);
  h[1] = xo(1) - measured(1);
  h[2] = xo(2) - measured(2);
  h[3] = xo(3) - measured(3);
  return h;
}

template <typename PointT>
gtsam::Vector Plane<PointT>::GetXf() const {
  gtsam::Vector g_v(4);
  g_v << a_, b_, c_, d_;
  return g_v;
}

// gtsam::Vector Plane::GetLinearState(const gtsam::Pose3& xr,
// 			      const omnimapper_msgs::WallFeature& measured,
// 			      boost::optional<Matrix&> dhbydxr,
// 			      boost::optional<Matrix&> dhbydxf) const{
//   gtsam::Vector xo = GetXo(xr);
//   gtsam::Vector p1 = gtsam::Vector_(3, measured.p1.x, measured.p1.y,
//   measured.p1.z); gtsam::Vector p2 = gtsam::Vector_(3, measured.p2.x,
//   measured.p2.y, measured.p2.z); gtsam::Vector b = (p2 - p1)/((p2-p1).norm())
//   ; double dx = p2(0) - p1(0); double dy = p2(1) - p1(1); double meas_d = (dy
//   * p1(0) - dx * p1(1))/
//     (sqrt(dx*dx + dy*dy));

//   gtsam::Vector normal = gtsam::Vector_(4,
// 				  -b(1),
// 				  b(0),
// 				  0.0,
// 				  meas_d);

//   gtsam::Vector h = Geth(xo,normal);
//   if(dhbydxr){
//     *dhbydxr = GetDh1(xr);
//   }
//   if(dhbydxf){
//     *dhbydxf = GetDh2(xr);
//   }
//   return h;
// }

/*

gtsam::Vector Plane::GetLinearState(const gtsam::Pose3& xr,
                                    const omnimapper_msgs::WallFeature&
measured, boost::optional<Matrix&> dhbydxr, boost::optional<Matrix&> dhbydxf)
const{ gtsam::Vector xo = GetXo(xr); gtsam::Vector p1 = gtsam::Vector_(3,
measured.p1.x, measured.p1.y, measured.p1.z); gtsam::Vector p2 =
gtsam::Vector_(3, measured.p2.x, measured.p2.y, measured.p2.z); gtsam::Vector b
= (p2 - p1)/((p2-p1).norm()) ; gtsam::Vector n = gtsam::Vector_(3, xo[0], xo[1],
xo[2]);

  gtsam::Vector p_bar = 0.5 * (p1 + p2);

  double dx = p2(0) - p1(0);
  double dy = p2(1) - p1(1);
  double meas_d = (dy * p1(0) - dx * p1(1))/
    (sqrt(dx*dx + dy*dy));

  double old_dist =
    xo[0] * p_bar[0] +
    xo[1] * p_bar[1] +
    xo[2] * p_bar[2] +
    xo[3];

  double dist = xo[3] - meas_d;
  //    printf("%lf is the measured dist, %lf is the predicted dist\n",
  //	   dist, xo[3]);
  gtsam::Vector h = gtsam::Vector_(2, b.dot(n), dist);//b.dot(n),
dist);//0.0);//, dist);//b.dot(n), dist); gtsam::Matrix detabydh; if (dhbydxr ||
dhbydxf) { detabydh = gtsam::zeros(2,4); detabydh(0,0) = b(0);//measured.p1.x;
    detabydh(0,1) = b(1);//measured.p1.y;
    detabydh(0,2) = b(2);//measured.p1.z;
    detabydh(0,3) = 0.0;

    detabydh(1,0) = 0.0;//p_bar[0];
    detabydh(1,1) = 0.0;//p_bar[1];
    detabydh(1,2) = 0.0;//p_bar[2];
    detabydh(1,3) = 1.0;
  }
  if (dhbydxr) {
    gtsam::Matrix dh1 = GetDh1(xr);
    *dhbydxr = detabydh * dh1;
  }
  if (dhbydxf) {
    gtsam::Matrix dh2 = GetDh2(xr);
    *dhbydxf = detabydh * dh2;
  }

  if (dhbydxr || dhbydxf) {
    std::stringstream ss;
    ss << "h: from laser line factor linear state";
    gtsam::print(h,ss.str());
    gtsam::print(detabydh,"detabydh");
  }
  return h;

}
*/
// gtsam::Vector Plane::GetLinearState(const gtsam::Pose3& xr,
// 			      const omnimapper_msgs::PlaneInfo& measured,
// 			      boost::optional<Matrix&> dhbydxr,
// 			      boost::optional<Matrix&> dhbydxf) const{

//   gtsam::Vector xo = GetXo(xr);
//   gtsam::Vector normal = gtsam::Vector_(4,
// 				  measured.model.values[0],
// 				  measured.model.values[1],
// 				  measured.model.values[2],
// 				  measured.model.values[3]);

//   gtsam::Vector h = Geth(xo,normal);
//   if(dhbydxr){
//     *dhbydxr = GetDh1(xr);
//   }
//   if(dhbydxf){
//     *dhbydxf = GetDh2(xr);
//   }
//   return h;
// }

// template <typename PointT> pcl::PointCloud<PointT>&
// Plane<PointT>::hull()
// {
//   // TODO: Check if we updated the coefficients, if so, generate the proper
//   rotation and apply it if (!((a_ == prev_a_) && (b_ == prev_b_) && (c_ ==
//   prev_c_) && (d_ == prev_d_)))
//   {
//     // We need to generate a rotation from the previous coefficients to this
//     one. Eigen::Vector3f new_normal (a_, b_, c_); Eigen::Vector3f prev_normal
//     (prev_a_, prev_b_, prev_c_); double angle = acos (prev_normal.dot
//     (new_normal)); Eigen::Vector3f axis = prev_normal.cross (new_normal);
//     Eigen::Affine3f transform;
//     transform = Eigen::AngleAxisf (angle, axis);
//     pcl::PointCloud<PointT> transformed_hull;
//     pcl::transformPointCloud (hull_, transformed_hull, transform);
//     hull_ = transformed_hull;
//   }
//   return (hull_);
// }

template <typename PointT>
gtsam::Vector Plane<PointT>::GetLinearState(
    const gtsam::Pose3& xr, const Plane& measured,
    boost::optional<Matrix&> dhbydxr, boost::optional<Matrix&> dhbydxf) const {
  gtsam::Vector xo = GetXo(xr);
  gtsam::Vector normal(4);
  normal << measured.a(), measured.b(), measured.c(), measured.d();

  gtsam::Vector h = Geth(xo, normal);
  if (dhbydxr) {
    *dhbydxr = GetDh1(xr);
  }
  if (dhbydxf) {
    *dhbydxf = GetDh2(xr);
  }
  return h;
}

template <typename PointT>
void Plane<PointT>::Retract(const Pose3& pose,
                            const gtsam::Plane<PointT>& plane) {
  // take just this hull one hull and project it back onto the model
  pcl::ModelCoefficients map_model;
  map_model.values.push_back(a_);
  map_model.values.push_back(b_);
  map_model.values.push_back(c_);
  map_model.values.push_back(d_);

  pcl::PointCloud<PointT> meas_hull_in_map;

  // tf::Transform posemap = Pose3ToTransform(pose);
  Eigen::Affine3f posemap = pose3ToTransform(pose);
  pcl::transformPointCloud(plane.hull_, meas_hull_in_map, posemap);

  pcl::PointCloud<PointT> meas_hull_on_map;
  pcl::ProjectInliers<PointT> proj1;
  proj1.setModelType(pcl::SACMODEL_PLANE);
  proj1.setInputCloud(
      boost::make_shared<pcl::PointCloud<PointT> >(meas_hull_in_map));
  proj1.setModelCoefficients(
      boost::make_shared<pcl::ModelCoefficients>(map_model));
  proj1.filter(meas_hull_on_map);

  hull_ = meas_hull_on_map;
}

// void Plane::Retract(const Pose3& pose, const gtsam::Plane& plane){
//   //take just this hull one hull and project it back onto the model
//   pcl::ModelCoefficients map_model;
//   map_model.values.push_back(a_);
//   map_model.values.push_back(b_);
//   map_model.values.push_back(c_);
//   map_model.values.push_back(d_);

//   pcl::PointCloud<PointT> meas_hull_in_map;

//   //gtsam::Vector rpy = pose.rotation().rpy();
//   //tf::Transform posemap = btTransform(tf::createQuaternionFromRPY(rpy[0],
//   //								    rpy[1],
//   //								    rpy[2]),
//   //					btVector3(pose.x(),pose.y(),pose.z()));
//   tf::Transform posemap = Pose3ToTransform(pose);
//   pcl_ros::transformPointCloud(plane.hull_,meas_hull_in_map,posemap);
//   meas_hull_in_map.header.frame_id = "/map";

//   pcl::PointCloud<PointT> meas_hull_on_map;
//   pcl::ProjectInliers<PointT> proj1;
//   proj1.setModelType(pcl::SACMODEL_PLANE);
//   proj1.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//   >(meas_hull_in_map));
//   proj1.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//   proj1.filter(meas_hull_on_map);

//   hull_ = meas_hull_on_map;
// }

// //This is a hack since PCL doesn't support boost::serialize
// void Plane::populateCloud(){
//   for(size_t i=0; i < out_hull.size(); i++){
//     //hull_ = pcl::PointCloud<PointT>();
//     hull_.header.frame_id = "/map";
//     hull_.header.stamp = ros::Time::now();
//     PointT pt((out_hull[i])[0],(out_hull[i])[1],(out_hull[i])[2]);
//     hull_.points.push_back(pt);
//   }
// }

// void Plane::Extend(const Pose3& pose, const gtsam::Plane& plane){
//   //Make a model coefficients from our map normal
//   pcl::ModelCoefficients map_model;
//   map_model.values.push_back(a_);
//   map_model.values.push_back(b_);
//   map_model.values.push_back(c_);
//   map_model.values.push_back(d_);

//   //reproject map hull, in case of updates
//   pcl::PointCloud<PointT> map_hull_on_map;
//   pcl::ProjectInliers<PointT> proj1;
//   proj1.setModelType(pcl::SACMODEL_PLANE);
//   proj1.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(hull_));
//   proj1.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//   proj1.filter(map_hull_on_map);
//   map_hull_on_map.header.frame_id = "/map";

//   //put this in the map frame
//   //gtsam::Vector rpy = pose.rotation().rpy();

//   pcl::PointCloud<PointT> meas_hull_in_map;
//   //tf::Transform posemap = btTransform(tf::createQuaternionFromRPY(rpy[0],
//   //								    rpy[1],
//   //								    rpy[2]),
//   //					btVector3(pose.x(),pose.y(),pose.z()));

//   meas_hull_in_map.header.frame_id = "/map";
//   tf::Transform posemap = Pose3ToTransform(pose);
//   pcl_ros::transformPointCloud(plane.hull_,meas_hull_in_map,posemap);

//   pcl::PointCloud<PointT> meas_hull_on_map;
//   pcl::ProjectInliers<PointT> proj;
//   proj.setModelType(pcl::SACMODEL_PLANE);
//   proj.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//   >(meas_hull_in_map));
//   proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//   proj.filter(meas_hull_on_map);
//   meas_hull_on_map.header.frame_id = "/map";

//   pcl::PointCloud<PointT> merged_cloud;
//   merged_cloud.header = plane.hull_.header;
//   merged_cloud.header.frame_id = "/map";
//   merged_cloud += map_hull_on_map;//hull_;
//   merged_cloud += meas_hull_on_map;

//   pcl::PointCloud<PointT> merged_hull;
//   if(concave_){
//     //project and merge inliers
//     pcl::PointCloud<PointT> map_inliers_on_map;
//     pcl::ProjectInliers<PointT> proj2;
//     proj2.setModelType(pcl::SACMODEL_PLANE);
//     proj2.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//     >(inliers_));
//     proj2.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//     proj2.filter(map_inliers_on_map);

//     pcl::PointCloud<PointT> meas_inliers_in_map;
//     pcl_ros::transformPointCloud(plane.inliers_,meas_inliers_in_map,posemap);

//     pcl::PointCloud<PointT> meas_inliers_on_map;
//     pcl::ProjectInliers<PointT> proj3;
//     proj3.setModelType(pcl::SACMODEL_PLANE);
//     proj3.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//     >(meas_inliers_in_map));
//     proj3.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//     proj3.filter(meas_inliers_on_map);

//     pcl::PointCloud<PointT> merged_cloud_full;
//     merged_cloud_full.header = hull_.header;
//     merged_cloud_full += map_inliers_on_map;
//     merged_cloud_full += meas_inliers_on_map;

//     inliers_ = merged_cloud_full;

//     pcl::ConcaveHull<PointT> chull;
//     std::vector<pcl::Vertices> pgons;
//     chull.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//     >(merged_cloud_full)); chull.setAlpha(0.15);
//     chull.reconstruct(merged_hull,pgons);
//   } else {
//     pcl::ConvexHull<PointT> chull;
//     chull.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//     >(merged_cloud)); chull.setDimension(2); chull.reconstruct(merged_hull);
//   }
//   hull_ = merged_hull;
// }

template <typename PointT>
void Plane<PointT>::Extend(const Pose3& pose,
                           const gtsam::Plane<PointT>& plane) {
  // Move the measured boundary to the map frame
  Eigen::Affine3f map_to_pose = pose3ToTransform(pose);
  Eigen::Affine3f pose_to_map = map_to_pose.inverse();

  // pcl::PointCloud<PointT> meas_boundary_map;
  // pcl::transformPointCloud (plane.hull (), meas_boundary_map, map_to_pose);
  gtsam::Point3 norm_in(plane.a(), plane.b(), plane.c());
  gtsam::Point3 norm_out = pose.rotation().rotate(norm_in);
  Eigen::Vector4d meas_coeffs_map(
      norm_out.x(), norm_out.y(), norm_out.z(),
      plane.a() * pose_to_map.translation().x() +
          plane.b() * pose_to_map.translation().y() +
          plane.c() * pose_to_map.translation().z() + plane.d());
  Eigen::Vector4d lm_coeffs_map(a_, b_, c_, d_);

  // Align the measured plane to the planar coefficients
  Eigen::Affine3d alignment_transform =
      planarAlignmentTransform(lm_coeffs_map, meas_coeffs_map);
  // pcl::PointCloud<PointT> meas_hull_aligned_map;
  // pcl::transformPointCloud (meas_hull_map, meas_hull_aligned_map, transform);

  // Compute the combined centroid for de-meaning the cloud
  Eigen::Vector4f lm_centroid;
  Eigen::Vector4f meas_centroid_local;
  pcl::compute3DCentroid(hull_, lm_centroid);
  pcl::compute3DCentroid(plane.hull(), meas_centroid_local);
  // Get the measurement centroid in the map frame
  Eigen::Vector3f meas_centroid_local_3f(
      meas_centroid_local[0], meas_centroid_local[1], meas_centroid_local[2]);
  Eigen::Vector3f meas_centroid_map = map_to_pose * meas_centroid_local_3f;

  Eigen::Vector3d lm_centroid_3d(lm_centroid[0], lm_centroid[1],
                                 lm_centroid[2]);
  Eigen::Vector3d meas_centroid_3d(meas_centroid_map[0], meas_centroid_map[1],
                                   meas_centroid_map[2]);
  Eigen::Vector3d combined_centroid =
      ((hull_.points.size() * lm_centroid_3d) +
       (plane.hull().points.size() * meas_centroid_3d)) /
      (hull_.points.size() + plane.hull().points.size());

  Eigen::Affine3d demean_transform = Eigen::Affine3d::Identity();
  demean_transform.translation() = -1.0 * combined_centroid;
  // pcl::PointCloud<PointT> origin_lm_hull;
  // pcl::PointCloud<PointT> origin_meas_hull;
  // pcl::transformPointCloud (hull_, origin_lm_hull, demean_transform);
  // pcl::transformPointCloud (meas_hull_aligned_map, origin_meas_hull,
  // demean_transform);

  // Rotate these to XY
  Eigen::Vector4d z_axis(0.0, 0.0, 1.0, 0.0);
  Eigen::Vector4d lm_coeffs_rot_only(a_, b_, c_, 0.0);
  Eigen::Affine3d z_alignment_transform =
      planarAlignmentTransform(z_axis, lm_coeffs_rot_only);

  Eigen::Affine3d transform_lm_to_xy = demean_transform * z_alignment_transform;
  Eigen::Affine3d transform_meas_to_xy =
      alignment_transform * demean_transform * z_alignment_transform;
  pcl::PointCloud<PointT> lm_xy;
  pcl::PointCloud<PointT> meas_xy;
  pcl::transformPointCloud(hull_, lm_xy, transform_lm_to_xy);
  pcl::transformPointCloud(plane.hull(), meas_xy, transform_meas_to_xy);

  // Polygon Union
  pcl::PointCloud<PointT> fused_xy;
  // bool worked = pcl::fusePlanarPolygonsXY (lm_xy, meas_xy, fused_xy);
  bool worked =
      omnimapper::fusePlanarPolygonsXY<PointT>(lm_xy, meas_xy, fused_xy);

  if (!worked) {
    printf("Error in plane::Extend! Merge failed!\n");
    exit(1);
    return;
  }

  // Move it back
  // pcl::transformPointCloud (fused_xy, hull_, transform_meas_to_xy.inverse
  // ());
  pcl::PointCloud<PointT> test_out;
  pcl::transformPointCloud(fused_xy, test_out, transform_meas_to_xy.inverse());

  bool verify_ptp_dist = true;
  if (verify_ptp_dist) {
    double orig_dist = 0.0;
    for (int i = 0; i < test_out.points.size(); i++) {
      double ptp_dist =
          fabs(a_ * test_out.points[i].x + b_ * test_out.points[i].y +
               c_ * test_out.points[i].z + d_);
      orig_dist += ptp_dist;
    }
    orig_dist = orig_dist / test_out.points.size();
    std::cout << "plane::Extend: dist: " << orig_dist << std::endl;
  }
}

template <typename PointT>
void Plane<PointT>::Extend2(const Pose3& pose,
                            const gtsam::Plane<PointT>& plane) {
  bool debug_extend2 = true;
  // TODO: should we do this in the local frame to avoid lever-arm effect (see
  // m-space paper)?
  Eigen::Affine3f map_to_pose =
      pose3ToTransform(pose);  //(pose.matrix ().cast<float>());
  Eigen::Affine3f pose_to_map = map_to_pose.inverse();
  // pcl::PointCloud<PointT> local_lm_hull = pcl::transformPointCloud (hull_,
  // local_lm_hull, pose); gtsam::Vector local_lm_coeffs = GetXo (pose);

  // Start by moving the hull to the map frame
  pcl::PointCloud<PointT> meas_hull_map;
  pcl::transformPointCloud(plane.hull(), meas_hull_map, map_to_pose);
  gtsam::Point3 norm_in(plane.a(), plane.b(), plane.c());
  gtsam::Point3 norm_out = pose.rotation().rotate(norm_in);
  Eigen::Vector4d meas_coeffs_map(
      norm_out.x(), norm_out.y(), norm_out.z(),
      plane.a() * pose_to_map.translation().x() +
          plane.b() * pose_to_map.translation().y() +
          plane.c() * pose_to_map.translation().z() + plane.d());
  Eigen::Vector4d lm_coeffs_map(a_, b_, c_, d_);

  if (debug_extend2) {
    // Confirm that these are both on xy and have a centroid near the origin
    Eigen::Vector4f debug_meas_centroid;
    Eigen::Matrix3f debug_meas_cov;

    pcl::computeMeanAndCovarianceMatrix(meas_hull_map, debug_meas_cov,
                                        debug_meas_centroid);

    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value_meas;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector_meas;
    pcl::eigen33(debug_meas_cov, eigen_value_meas, eigen_vector_meas);
    Eigen::Vector4f coeffs_meas;
    coeffs_meas[0] = eigen_vector_meas[0];
    coeffs_meas[1] = eigen_vector_meas[1];
    coeffs_meas[2] = eigen_vector_meas[2];
    coeffs_meas[3] = 0;
    coeffs_meas[3] = -1 * coeffs_meas.dot(debug_meas_centroid);

    std::cout << "lm coeffs: " << a_ << ", " << b_ << ", " << c_ << ", " << d_
              << std::endl;
    std::cout << "meas coeffs from hull: " << coeffs_meas << std::endl;
    std::cout << "meas coeffs: " << meas_coeffs_map << std::endl;

    std::cout << "meas origin ctroid: " << debug_meas_centroid << std::endl;
  }

  // hull_ += meas_hull_map;
  // return;

  std::cout << "lm_coeffs_map: " << lm_coeffs_map << std::endl;
  std::cout << "meas_coeffs_map: " << meas_coeffs_map << std::endl;

  // Perform rotation to align the measurement to the landmark's coefficients
  Eigen::Vector3d lm_norm(a_, b_, c_);
  Eigen::Vector3d meas_norm(meas_coeffs_map[0], meas_coeffs_map[1],
                            meas_coeffs_map[2]);
  printf("meas_norm norm: %lf\n", meas_norm.norm());
  double angle1 = acos(meas_norm.dot(lm_norm));
  Eigen::Vector3d axis1 = meas_norm.cross(lm_norm);
  axis1.normalize();
  Eigen::Affine3d transform1;
  transform1 = Eigen::AngleAxisd(angle1, axis1);
  Eigen::Vector3d translation_part1 = lm_norm * (meas_coeffs_map[3] - d_);
  transform1.translation() = translation_part1;
  pcl::PointCloud<PointT> meas_hull_aligned_map1;
  pcl::transformPointCloud(meas_hull_map, meas_hull_aligned_map1, transform1);

  double angle = acos(meas_norm.dot(lm_norm));
  Eigen::Vector3d axis = meas_norm.cross(lm_norm);
  axis.normalize();
  Eigen::Affine3d transform;
  if ((pcl_isfinite(angle)) && (pcl_isfinite(axis[0])) &&
      (pcl_isfinite(axis[1])) && (pcl_isfinite(axis[2]))) {
    transform = Eigen::AngleAxisd(angle, axis);
  } else {
    transform = Eigen::Affine3d::Identity();
  }
  Eigen::Vector3d translation_part = lm_norm * (meas_coeffs_map[3] - d_);
  transform.translation() = translation_part;
  pcl::PointCloud<PointT> meas_hull_aligned_map;
  pcl::transformPointCloud(meas_hull_map, meas_hull_aligned_map, transform);

  if (debug_extend2) {
    // Confirm that these are both on xy and have a centroid near the origin
    Eigen::Vector4f debug_lm_centroid;
    Eigen::Vector4f debug_meas_centroid;
    Eigen::Matrix3f debug_lm_cov;
    Eigen::Matrix3f debug_meas_cov;

    pcl::computeMeanAndCovarianceMatrix(meas_hull_aligned_map1, debug_lm_cov,
                                        debug_lm_centroid);
    pcl::computeMeanAndCovarianceMatrix(meas_hull_aligned_map, debug_meas_cov,
                                        debug_meas_centroid);

    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value_lm;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector_lm;
    pcl::eigen33(debug_lm_cov, eigen_value_lm, eigen_vector_lm);
    Eigen::Vector4f coeffs_lm;
    coeffs_lm[0] = eigen_vector_lm[0];
    coeffs_lm[1] = eigen_vector_lm[1];
    coeffs_lm[2] = eigen_vector_lm[2];
    coeffs_lm[3] = 0;
    coeffs_lm[3] = -1 * coeffs_lm.dot(debug_lm_centroid);

    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value_meas;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector_meas;
    pcl::eigen33(debug_meas_cov, eigen_value_meas, eigen_vector_meas);
    Eigen::Vector4f coeffs_meas;
    coeffs_meas[0] = eigen_vector_meas[0];
    coeffs_meas[1] = eigen_vector_meas[1];
    coeffs_meas[2] = eigen_vector_meas[2];
    coeffs_meas[3] = 0;
    coeffs_meas[3] = -1 * coeffs_meas.dot(debug_meas_centroid);

    std::cout << "lm coeffs: " << a_ << ", " << b_ << ", " << c_ << ", " << d_
              << std::endl;
    std::cout << "meas1 coeffs: " << coeffs_lm << std::endl;
    std::cout << "meas coeffs: " << coeffs_meas << std::endl;

    // pcl::compute3DCentroid (origin_xy_lm_hull, debug_lm_centroid);
    // pcl::compute3DCentroid (origin_xy_meas_hull, debug_meas_centroid);
    // std::cout << "combined map ctroid: " << combined_centroid3d << std::endl;
    std::cout << "lm origin ctroid: " << debug_lm_centroid << std::endl;
    std::cout << "meas origin ctroid: " << debug_meas_centroid << std::endl;
  }

  if (debug_extend2) {
    // now check the point to plane distance of each point from the landmark
    double hull1_dist = 0.0;
    double hull2_dist = 0.0;
    double hull3_dist = 0.0;
    for (int i = 0; i < meas_hull_aligned_map.points.size(); i++) {
      double ptp_dist1 = fabs(a_ * meas_hull_aligned_map.points[i].x +
                              b_ * meas_hull_aligned_map.points[i].y +
                              c_ * meas_hull_aligned_map.points[i].z + d_);
      double ptp_dist2 = fabs(a_ * meas_hull_aligned_map1.points[i].x +
                              b_ * meas_hull_aligned_map1.points[i].y +
                              c_ * meas_hull_aligned_map1.points[i].z + d_);
      hull1_dist += ptp_dist1;
      hull2_dist += ptp_dist2;
      // if (ptp_dist >= 0.001)
      //{
      // printf ("Error in Planeextend!  PTP dist not zero after aligning the
      // meas: %lf\n", ptp_dist); printf ("Point: %lf %lf %lf\n",
      // meas_hull_aligned_map.points[i].x, meas_hull_aligned_map.points[i].y,
      // meas_hull_aligned_map.points[i].z);
      //}
    }
    for (int i = 0; i < hull_.points.size(); i++) {
      double ptp_dist3 = fabs(a_ * hull_.points[i].x + b_ * hull_.points[i].y +
                              c_ * hull_.points[i].z + d_);
      hull3_dist += ptp_dist3;
    }
    hull1_dist = hull1_dist / meas_hull_aligned_map.points.size();
    hull2_dist = hull2_dist / meas_hull_aligned_map.points.size();
    hull3_dist = hull3_dist / hull_.points.size();
    printf("Hull1 dist: %lf Hull2 dist: %lf Hull3 dist: %lf\n", hull1_dist,
           hull2_dist, hull3_dist);
  }

  // hull_ += meas_hull_aligned_map;
  // return;

  // Now get the centroid of both clouds so we can de-mean them
  // TODO: make efficient
  pcl::PointCloud<PointT> combined_cloud;
  combined_cloud += hull_;
  combined_cloud += meas_hull_aligned_map;
  Eigen::Vector4f combined_centroid;
  pcl::compute3DCentroid(combined_cloud, combined_centroid);
  Eigen::Vector3d combined_centroid3d(
      combined_centroid[0], combined_centroid[1], combined_centroid[2]);

  // if (debug_extend2)
  // {
  //   hull_ = combined_cloud;
  //   return;
  // }

  // de-mean
  Eigen::Affine3d demean_transform = Eigen::Affine3d::Identity();
  demean_transform.translation() = -1.0 * combined_centroid3d;
  pcl::PointCloud<PointT> origin_lm_hull;
  pcl::PointCloud<PointT> origin_meas_hull;
  pcl::transformPointCloud(hull_, origin_lm_hull, demean_transform);
  pcl::transformPointCloud(meas_hull_aligned_map, origin_meas_hull,
                           demean_transform);

  // Rotate to xy
  Eigen::Vector3d z_axis(0.0, 0.0, 1.0);
  double lm_to_z_angle = acos(lm_norm.dot(z_axis));  // z_axis.dot (lm_norm);
  Eigen::Vector3d lm_to_z_axis =
      lm_norm.cross(z_axis);  // z_axis.cross (lm_norm);
  lm_to_z_axis.normalize();
  Eigen::Affine3d lm_to_z_transform;
  lm_to_z_transform = Eigen::AngleAxisd(lm_to_z_angle, lm_to_z_axis);
  pcl::PointCloud<PointT> origin_xy_lm_hull;
  pcl::PointCloud<PointT> origin_xy_meas_hull;
  pcl::transformPointCloud(origin_lm_hull, origin_xy_lm_hull,
                           lm_to_z_transform);
  pcl::transformPointCloud(origin_meas_hull, origin_xy_meas_hull,
                           lm_to_z_transform);

  if (debug_extend2) {
    // Confirm that these are both on xy and have a centroid near the origin
    Eigen::Vector4f debug_lm_centroid;
    Eigen::Vector4f debug_meas_centroid;
    Eigen::Matrix3f debug_lm_cov;
    Eigen::Matrix3f debug_meas_cov;

    pcl::computeMeanAndCovarianceMatrix(origin_xy_lm_hull, debug_lm_cov,
                                        debug_lm_centroid);
    pcl::computeMeanAndCovarianceMatrix(origin_xy_meas_hull, debug_meas_cov,
                                        debug_meas_centroid);

    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value_lm;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector_lm;
    pcl::eigen33(debug_lm_cov, eigen_value_lm, eigen_vector_lm);
    Eigen::Vector4f coeffs_lm;
    coeffs_lm[0] = eigen_vector_lm[0];
    coeffs_lm[1] = eigen_vector_lm[1];
    coeffs_lm[2] = eigen_vector_lm[2];
    coeffs_lm[3] = 0;
    coeffs_lm[3] = -1 * coeffs_lm.dot(debug_lm_centroid);

    EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value_meas;
    EIGEN_ALIGN16 Eigen::Vector3f eigen_vector_meas;
    pcl::eigen33(debug_meas_cov, eigen_value_meas, eigen_vector_meas);
    Eigen::Vector4f coeffs_meas;
    coeffs_meas[0] = eigen_vector_meas[0];
    coeffs_meas[1] = eigen_vector_meas[1];
    coeffs_meas[2] = eigen_vector_meas[2];
    coeffs_meas[3] = 0;
    coeffs_meas[3] = -1 * coeffs_meas.dot(debug_meas_centroid);

    std::cout << "lm coeffs: " << coeffs_lm << std::endl;
    std::cout << "meas coeffs: " << coeffs_meas << std::endl;

    // pcl::compute3DCentroid (origin_xy_lm_hull, debug_lm_centroid);
    // pcl::compute3DCentroid (origin_xy_meas_hull, debug_meas_centroid);
    std::cout << "combined map ctroid: " << combined_centroid3d << std::endl;
    std::cout << "lm origin ctroid: " << debug_lm_centroid << std::endl;
    std::cout << "meas origin ctroid: " << debug_meas_centroid << std::endl;
  }

  pcl::PointCloud<PointT> origin_xy_fused_hull;
  // bool worked = pcl::fusePlanarPolygonsXY (origin_xy_lm_hull,
  // origin_xy_meas_hull, origin_xy_fused_hull);
  bool worked = omnimapper::fusePlanarPolygonsXY<PointT>(
      origin_xy_lm_hull, origin_xy_meas_hull, origin_xy_fused_hull);

  printf("Fuse Result: orig: %lu meas: %lu fused: %lu\n",
         origin_xy_lm_hull.points.size(), origin_xy_meas_hull.points.size(),
         origin_xy_fused_hull.points.size());

  if (!worked) {
    printf("Error fusing polygons in inside planeextend!!\n");
    return;
  }

  if (debug_extend2) {
    printf("lm_hull: %lu meas_hull: %lu fused_hull: %lu\n",
           origin_xy_lm_hull.points.size(), origin_xy_meas_hull.points.size(),
           origin_xy_fused_hull.points.size());
  }

  // Simplify boundary
  pcl::PointCloud<PointT> origin_xy_fused_approx_hull;
  typename pcl::PointCloud<PointT>::VectorType& xy_fused_points =
      origin_xy_fused_hull.points;
  typename pcl::PointCloud<PointT>::VectorType& xy_fused_approx_points =
      origin_xy_fused_approx_hull.points;

  pcl::approximatePolygon2D<PointT>(xy_fused_points, xy_fused_approx_points,
                                    0.005, false, true);
  printf("approximatePolygon2D: orig poly: %lu new poly: %lu\n",
         xy_fused_points.size(), xy_fused_approx_points.size());

  // Rotate it back
  pcl::PointCloud<PointT> fused_rotated_back;
  Eigen::Affine3d rot_inv = lm_to_z_transform.inverse();
  // pcl::transformPointCloud (origin_xy_fused_hull, fused_rotated_back,
  // rot_inv);
  pcl::transformPointCloud(origin_xy_fused_approx_hull, fused_rotated_back,
                           rot_inv);

  // Re-mean it
  pcl::PointCloud<PointT> fused_on_map;
  Eigen::Affine3d remean_transform = demean_transform.inverse();
  pcl::transformPointCloud(fused_rotated_back, fused_on_map, remean_transform);

  // Store it
  hull_ = fused_on_map;

  if (debug_extend2) {
    // now check the point to plane distance of each point from the landmark
    for (int i = 0; i < hull_.points.size(); i++) {
      double ptp_dist = fabs(a_ * hull_.points[i].x + b_ * hull_.points[i].y +
                             c_ * hull_.points[i].z + d_);
      if (ptp_dist >= 0.001) {
        printf("Error in Planeextend!  PTP dist not zero after extend: %lf\n",
               ptp_dist);
        printf("Point: %lf %lf %lf\n", hull_.points[i].x, hull_.points[i].y,
               hull_.points[i].z);
      }
    }
  }
}

// template <typename PointT>
// void Plane<PointT>::Extend(const Pose3& pose, const gtsam::Plane<PointT>&
// plane){
//   //Make a model coefficients from our map normal
//   pcl::ModelCoefficients map_model;
//   map_model.values.push_back(a_);
//   map_model.values.push_back(b_);
//   map_model.values.push_back(c_);
//   map_model.values.push_back(d_);

//   //reproject map hull, in case of updates
//   // TODO: this should be a rotation, not a projection.
//   pcl::PointCloud<PointT> map_hull_on_map;
//   pcl::ProjectInliers<PointT> proj1;
//   proj1.setModelType(pcl::SACMODEL_PLANE);
//   proj1.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> >(hull_));
//   proj1.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//   proj1.filter(map_hull_on_map);
//   map_hull_on_map.header.frame_id = "/map";

//   //put this in the map frame
//   //gtsam::Vector rpy = pose.rotation().rpy();

//   pcl::PointCloud<PointT> meas_hull_in_map;
//   //tf::Transform posemap = btTransform(tf::createQuaternionFromRPY(rpy[0],
//   //								    rpy[1],
//   //								    rpy[2]),
//   //					btVector3(pose.x(),pose.y(),pose.z()));

//   meas_hull_in_map.header.frame_id = "/map";
//   //tf::Transform posemap = Pose3ToTransform(pose);
//   Eigen::Affine3f posemap = pose3ToTransform(pose);
//   pcl::transformPointCloud(plane.hull_,meas_hull_in_map,posemap);

//   pcl::PointCloud<PointT> meas_hull_on_map;
//   pcl::ProjectInliers<PointT> proj;
//   proj.setModelType(pcl::SACMODEL_PLANE);
//   proj.setInputCloud(boost::make_shared<pcl::PointCloud<PointT>
//   >(meas_hull_in_map));
//   proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//   proj.filter(meas_hull_on_map);
//   meas_hull_on_map.header.frame_id = "/map";

//   pcl::PointCloud<PointT> merged_cloud;
//   merged_cloud.header = plane.hull_.header;
//   merged_cloud.header.frame_id = "/map";
//   merged_cloud += map_hull_on_map;//hull_;
//   merged_cloud += meas_hull_on_map;

//   pcl::PointCloud<PointT> merged_hull;

//   //TEST
//   Eigen::Vector4f vec_model (map_model.values[0], map_model.values[1],
//   map_model.values[2], map_model.values[3]); pcl::PlanarPolygon<PointT>
//   map_region (map_hull_on_map.points, vec_model); pcl::PlanarPolygon<PointT>
//   meas_region (meas_hull_on_map.points, vec_model);
//   pcl::PlanarPolygon<PointT> fused_region;
//   pcl::PointCloud<PointT> xy1;
//   pcl::PointCloud<PointT> xy2;

//   //pcl::PlanarPolygon<Point> approx_map;
//   //pcl::PlanarPolygon<Point> approx_meas;
//   //pcl::approximatePolygon (map_region, approx_map, 0.005, false);
//   //pcl::approximatePolygon (meas_region, approx_meas, 0.005, false);

//   bool fused_worked = pcl::fusePlanarPolygons (map_region, meas_region,
//   fused_region, vec_model, xy1, xy2);

//   if (!fused_worked)
//   {
//     printf ("ERROR FUSING INSIDE PLANE!\n\n\n\n\n\n\n");
//   }

//   //TEST

//   /*
//   if(concave_){
//     //project and merge inliers
//     pcl::PointCloud<Point> map_inliers_on_map;
//     pcl::ProjectInliers<Point> proj2;
//     proj2.setModelType(pcl::SACMODEL_PLANE);
//     proj2.setInputCloud(boost::make_shared<pcl::PointCloud<Point>
//     >(inliers_));
//     proj2.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//     proj2.filter(map_inliers_on_map);

//     pcl::PointCloud<Point> meas_inliers_in_map;
//     pcl::transformPointCloud(plane.inliers_,meas_inliers_in_map,posemap);

//     pcl::PointCloud<Point> meas_inliers_on_map;
//     pcl::ProjectInliers<Point> proj3;
//     proj3.setModelType(pcl::SACMODEL_PLANE);
//     proj3.setInputCloud(boost::make_shared<pcl::PointCloud<Point>
//     >(meas_inliers_in_map));
//     proj3.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
//     proj3.filter(meas_inliers_on_map);

//     pcl::PointCloud<Point> merged_cloud_full;
//     merged_cloud_full.header = hull_.header;
//     merged_cloud_full += map_inliers_on_map;
//     merged_cloud_full += meas_inliers_on_map;

//     inliers_ = merged_cloud_full;

//     pcl::ConcaveHull<Point> chull;
//     std::vector<pcl::Vertices> pgons;
//     chull.setInputCloud(boost::make_shared<pcl::PointCloud<Point>
//     >(merged_cloud_full)); chull.setAlpha(0.15);
//     chull.reconstruct(merged_hull,pgons);
//   } else {
//   */
//   //pcl::ConvexHull<Point> chull;
//   // chull.setInputCloud(boost::make_shared<pcl::PointCloud<Point>
//   >(merged_cloud));
//   //  chull.reconstruct(merged_hull);
//     //}
//   //hull_ = merged_hull;

//   // std::vector<int> hull_inds;
//   // getConvexHull2D(merged_cloud.points,hull_inds);
//   // pcl::ExtractIndices<Point> extract;
//   // pcl::PointIndices hull_pi;
//   // hull_pi.indices = hull_inds;
//   // extract.setInputCloud(boost::make_shared<pcl::PointCloud<Point>
//   >(merged_cloud));
//   // extract.setIndices(boost::make_shared<pcl::PointIndices>(hull_pi));
//   // extract.filter(merged_hull);
//   //hull_ = merged_hull;
//   if (fused_worked)
//     hull_.points = fused_region.getContour ();
//   else
//     hull_ = map_hull_on_map;

// }
/* ************************************************************************* */
}  // namespace gtsam

// template class gtsam::Plane<pcl::PointXYZ>;
