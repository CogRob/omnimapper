#include <glog/logging.h>
#include <omnimapper/BoundedPlane3.h>
#include <omnimapper/geometry.h>
#include <omnimapper/transform_helpers.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/io/pcd_io.h>

#include <omnimapper/impl/geometry.hpp>

/* ************************************************************************* */
/// The print fuction
template <typename PointT>
void omnimapper::BoundedPlane3<PointT>::print(const std::string& s) const {
  gtsam::Vector coeffs = planeCoefficients();
  std::cout << s << " : " << coeffs << std::endl;
}

template <typename PointT>
void omnimapper::BoundedPlane3<PointT>::reprojectBoundary() {}

template <typename PointT>
omnimapper::BoundedPlane3<PointT> omnimapper::BoundedPlane3<PointT>::retract(
    const gtsam::Vector& v) const {
  boost::lock_guard<boost::mutex> lock(*plane_mutex_);
  LOG(INFO) << "BoundedPlane3: retracting: " << v;

  // Retract coefficients
  gtsam::Vector2 n_v(v(0), v(1));
  gtsam::Unit3 n_retracted = n_.retract(n_v);
  double d_retracted = d_ + v(2);

  // Retract the boundary too.
  Eigen::Vector3d n_retracted_unit = n_retracted.point3().vector();
  Eigen::Vector4d new_coeffs(n_retracted_unit[0], n_retracted_unit[1],
                             n_retracted_unit[2], d_retracted);
  Eigen::Vector4d old_coeffs = planeCoefficients();
  // Eigen::Affine3d transform = planarAlignmentTransform (old_coeffs,
  // new_coeffs);
  Eigen::Affine3d transform = PlanarAlignmentTransform(new_coeffs, old_coeffs);

  LOG(INFO) << "BoundedPlane3: transform translation: "
            << transform.translation();

  CloudPtr new_boundary(new Cloud());
  pcl::transformPointCloud(*boundary_, *new_boundary, transform);
  // pcl::copyPointCloud(*new_boundary, *boundary_);
  // pcl::transformPointCloud(*boundary_, *boundary_, transform);
  //*boundary_ = *new_boundary;
  // boundary_.swap(new_boundary);

  for (std::size_t i = 0; i < new_boundary->points.size(); i++) {
    // printf ("Meas Boundary Map: Boundary Point: %lf %lf %lf\n",
    //   meas_boundary_map->points[i].x ,
    //   meas_boundary_map->points[i].y ,
    //   meas_boundary_map->points[i].z );

    double ptp_dist =
        fabs(new_coeffs[0] * new_boundary->points[i].x +
             new_coeffs[1] * new_boundary->points[i].y +
             new_coeffs[2] * new_boundary->points[i].z + new_coeffs[3]);
    if (ptp_dist > 0.001) {
      LOG(INFO) << "ERROR: Retract fail: Point is " << ptp_dist
                << "from plane.";
      exit(1);
    }
  }

  // return (omnimapper::BoundedPlane3<PointT>(n_retracted, d_retracted,
  // boundary_, plane_mutex_));
  return omnimapper::BoundedPlane3<PointT>(
      n_retracted, d_retracted,
      std::make_pair(new_boundary, boost::make_shared<boost::mutex>()));
  // return (omnimapper::BoundedPlane3<PointT>(n_retracted, d_retracted,
  // boundary_));
}

template <typename PointT>
gtsam::Vector omnimapper::BoundedPlane3<PointT>::localCoordinates(
    const omnimapper::BoundedPlane3<PointT>& y) const {
  gtsam::Vector n_local = n_.localCoordinates(y.n_);
  double d_local = d_ - y.d_;
  return gtsam::Vector(3) << n_local(0), n_local(1), -d_local;
}

template <typename PointT>
gtsam::Vector omnimapper::BoundedPlane3<PointT>::planeCoefficients() const {
  gtsam::Vector unit_vec = n_.point3().vector();
  return (gtsam::Vector(4) << unit_vec[0], unit_vec[1], unit_vec[2], d_);
}

template <typename PointT>
omnimapper::BoundedPlane3<PointT> omnimapper::BoundedPlane3<PointT>::Transform(
    const omnimapper::BoundedPlane3<PointT>& plane, const gtsam::Pose3& xr,
    boost::optional<gtsam::Matrix&> Hr, boost::optional<gtsam::Matrix&> Hp) {
  // TODO: this should be renamed, as it does not transform the planar boundary,
  // only the coefficients.
  gtsam::Matrix n_hr;
  gtsam::Matrix n_hp;
  gtsam::Unit3 n_rotated = xr.rotation().unrotate(plane.n_, n_hr, n_hp);

  gtsam::Vector n_unit = plane.n_.point3().vector();
  gtsam::Vector unit_vec = n_rotated.point3().vector();
  double pred_d = n_unit.dot(xr.translation().vector()) + plane.d_;
  // OrientedPlane3 transformed_plane (unit_vec (0), unit_vec (1), unit_vec (2),
  // pred_d);

  CloudPtr transformed_boundary(new Cloud());
  Eigen::Affine3f transform = Pose3ToTransform(xr);
  // pcl::transformPointCloud (*(plane.boundary_), *transformed_boundary,
  // transform);

  // TODO(shengye): Here we actually have a deep copy. Profile and if this the
  // the bottleneck, optmize it with non-deep copy construction.
  boost::lock_guard<boost::mutex> lock_plane(*(plane.boundary().second));
  BoundedPlane3<PointT> transformed_plane(unit_vec(0), unit_vec(1), unit_vec(2),
                                          pred_d, plane.boundary().first);
  // BoundedPlane3<PointT> transformed_plane (unit_vec (0), unit_vec (1),
  // unit_vec (2), pred_d, transformed_boundary);

  if (Hr) {
    *Hr = gtsam::zeros(3, 6);
    (*Hr).block<2, 3>(0, 0) = n_hr;
    (*Hr).block<1, 3>(2, 3) = unit_vec;
  }
  if (Hp) {
    gtsam::Vector xrp = xr.translation().vector();
    gtsam::Matrix n_basis = plane.n_.basis();
    gtsam::Vector hpp = n_basis.transpose() * xrp;
    *Hp = gtsam::zeros(3, 3);
    (*Hp).block<2, 2>(0, 0) = n_hp;
    (*Hp).block<1, 2>(2, 0) = hpp;
    (*Hp)(2, 2) = 1;
  }

  return (transformed_plane);
}

template <typename PointT>
Eigen::Vector4d omnimapper::BoundedPlane3<PointT>::TransformCoefficients(
    const omnimapper::BoundedPlane3<PointT>& plane, const gtsam::Pose3& xr) {
  gtsam::Unit3 n_rotated = xr.rotation().unrotate(plane.n_);
  gtsam::Vector n_rotated_unit = n_rotated.point3().vector();
  gtsam::Vector n_unit = plane.n_.point3().vector();
  double pred_d = n_unit.dot(xr.translation().vector()) + plane.d_;
  Eigen::Vector4d result(n_rotated_unit[0], n_rotated_unit[1],
                         n_rotated_unit[2], pred_d);

  // if (xr.translation ().vector ().dot (n_rotated_unit) > 0)
  // {
  //   result = -result;
  //   printf ("BoundedPlane3: flipping normal on transform coeffs!\n");
  // }

  return (result);
}

/* ************************************************************************* */
template <typename PointT>
gtsam::Vector omnimapper::BoundedPlane3<PointT>::error(
    const omnimapper::BoundedPlane3<PointT>& plane) const {
  gtsam::Vector n_error = -n_.localCoordinates(plane.n_);

  if (!(std::isfinite(n_error[0]) && std::isfinite(n_error[1]))) {
    n_.print("BoundedPlane3 NaN debug, n_");
    plane.n_.print("BoundedPlane3 NaN debug, plane.n_");
    LOG(INFO) << n_error.size() << ", " << n_error[0] << ", " << n_error[1];

    LOG(FATAL) << "BoundedPlane3: ERROR: Got NaN error on local coords!";
    // exit (3);
  }

  double d_error = d_ - plane.d_;
  LOG(INFO) << "BoundedPlane3: error: " << n_error << " " << d_error;
  return (gtsam::Vector(3) << n_error(0), n_error(1), d_error);
}

template <typename PointT>
void omnimapper::BoundedPlane3<PointT>::extendBoundary(
    const gtsam::Pose3& pose, const BoundedPlane3<PointT>& plane) const {
  boost::lock_guard<boost::mutex> lock(*plane_mutex_);
  CloudConstMutexPtrPair meas_boundary_mutex_pair = plane.boundary();
  boost::lock_guard<boost::mutex> meas_lock(*(meas_boundary_mutex_pair.second));

  CloudConstPtr meas_boundary = meas_boundary_mutex_pair.first;

  Eigen::Vector4d z_axis(0.0, 0.0, 1.0, 0.0);
  // return;

  bool check_input = false;

  //  Move map cloud to the pose
  Eigen::Affine3d map_to_pose = Pose3ToTransform(pose).cast<double>();
  Eigen::Affine3d pose_to_map = map_to_pose.inverse();
  Eigen::Vector4d lm_coeffs = TransformCoefficients(*this, pose);
  Eigen::Affine3d lm_to_xy = PlanarAlignmentTransform(z_axis, lm_coeffs);
  Eigen::Affine3d xy_to_lm = PlanarAlignmentTransform(lm_coeffs, z_axis);
  Eigen::Affine3d lm_combined =
      lm_to_xy * pose_to_map;  // map_to_pose * lm_to_xy;
  Eigen::Affine3d lm_combined_inv = lm_combined.inverse();
  // Eigen::Affine3d lm_combined_inv = map_to_pose * xy_to_lm;
  CloudPtr map_xy(new Cloud());
  LOG(INFO) << "BoundedPlane3: transforming landmark to pose.";
  pcl::transformPointCloud(*boundary_, *map_xy, lm_combined);
  LOG(INFO) << "BoundedPlane3: transformed.";

  // Move the measurement to the xy plane
  Eigen::Vector4d meas_coeffs = plane.planeCoefficients();
  Eigen::Affine3d meas_to_xy = PlanarAlignmentTransform(z_axis, meas_coeffs);
  // Eigen::Affine3d meas_to_xy = planarAlignmentTransform(meas_coeffs, z_axis);
  CloudPtr meas_xy(new Cloud());
  LOG(INFO) << "BoundedPlane3: transforming measurement to xy.";
  pcl::transformPointCloud(*meas_boundary, *meas_xy, meas_to_xy);
  LOG(INFO) << "BoundedPlane3: transformed.  Merging.";

  // TEST
  if (check_input) {
    Eigen::Vector4d map_coeffs = planeCoefficients();
    for (std::size_t i = 0; i < boundary_->points.size(); i++) {
      // printf ("Internal Boundary: Boundary Point: %lf %lf %lf\n",
      // boundary_->points[i].x ,
      // boundary_->points[i].y ,
      // boundary_->points[i].z );

      double ptp_dist =
          fabs(map_coeffs[0] * boundary_->points[i].x +
               map_coeffs[1] * boundary_->points[i].y +
               map_coeffs[2] * boundary_->points[i].z + map_coeffs[3]);
      if (ptp_dist > 0.01) {
        LOG(INFO) << "ERROR: Boundary: Point is " << ptp_dist << "from plane.";
        exit(1);
      }
    }

    for (std::size_t i = 0; i < meas_xy->points.size(); i++) {
      // printf ("MeasXY: Boundary Point: %lf %lf %lf\n",
      // meas_xy->points[i].x ,
      // meas_xy->points[i].y ,
      // meas_xy->points[i].z );

      double ptp_dist = fabs(z_axis[0] * meas_xy->points[i].x +
                             z_axis[1] * meas_xy->points[i].y +
                             z_axis[2] * meas_xy->points[i].z + z_axis[3]);
      if (ptp_dist > 0.001) {
        LOG(INFO) << "ERROR: MeasXY: Point is " << ptp_dist << "from plane.";
        exit(1);
      }
    }

    for (std::size_t i = 0; i < map_xy->points.size(); i++) {
      // printf ("MapXY: Boundary Point: %lf %lf %lf\n",
      // map_xy->points[i].x ,
      // map_xy->points[i].y ,
      // map_xy->points[i].z );

      double ptp_dist = fabs(z_axis[0] * map_xy->points[i].x +
                             z_axis[1] * map_xy->points[i].y +
                             z_axis[2] * map_xy->points[i].z + z_axis[3]);
      if (ptp_dist > 0.001) {
        LOG(INFO) << "ERROR: MapXY: Point is " << ptp_dist << "from plane.";
        exit(1);
      }
    }

    bool boundary_intersects = boost::geometry::intersects(boundary_->points);
    if (boundary_intersects) {
      LOG(INFO) << "BoundedPlane3: map boundary intersects!";
      exit(1);
    }
    bool meas_boundary_intersects =
        boost::geometry::intersects(meas_boundary->points);
    if (meas_boundary_intersects)
      LOG(INFO) << "BoundedPlane3: meas boundary intersects!";

    LOG(INFO) << "BoundedPlane3: Attempting to merge meas_xy ("
              << meas_xy->points.size() << "with map_xy"
              << map_xy->points.size();
    bool meas_xy_intersect = boost::geometry::intersects(meas_xy->points);
    LOG_IF(INFO, meas_xy_intersect) << "BoundedPlane3: Meas_xy_intersects!";
    bool map_xy_intersect = boost::geometry::intersects(map_xy->points);
    LOG_IF(INFO, map_xy_intersect) << "BoundedPlane3: Map_xy intersects!";
  }

  double map_area = boost::geometry::area(map_xy->points);
  double meas_area = boost::geometry::area(meas_xy->points);

  char meas_name[2048];
  char lm_name[2048];
  sprintf(meas_name, "meas.pcd");
  sprintf(lm_name, "lm.pcd");
  pcl::io::savePCDFileBinaryCompressed(meas_name, *meas_xy);
  pcl::io::savePCDFileBinaryCompressed(lm_name, *map_xy);

  // TEST

  // Fuse
  CloudPtr merged_xy(new Cloud());
  bool worked = omnimapper::FusePlanarPolygonsConvexXY<PointT>(
      *meas_xy, *map_xy, *merged_xy);
  if (!worked) {
    LOG(INFO) << "BoundedPlane3: Error inside extend!";
    return;
    // exit(1);
  }

  LOG(INFO) << "BoundedPlane3: Merged: map: " << map_xy->points.size()
            << " meas:" << meas_xy->points.size()
            << " combined:" << merged_xy->points.size();
  double merged_area = boost::geometry::area(merged_xy->points);
  if ((merged_area < meas_area) || (merged_area < map_area)) {
    LOG(INFO) << "BoundedPlane3: meas_area: " << meas_area
              << " map_area: " << map_area << " merged_area: " << merged_area;
    // exit(1);
  }

  bool merged_intersects = boost::geometry::intersects(merged_xy->points);
  if (merged_intersects) {
    LOG(INFO) << "BoundedPlane3: merged intersects!";
    char merged_name[2048];
    sprintf(merged_name, "merged.pcd");
    pcl::io::savePCDFileBinaryCompressed(merged_name, *merged_xy);
    exit(1);
  }

  // Now move it back to the map
  CloudPtr merged_map(new Cloud());
  // Eigen::Affine3d lm_combined_inv = lm_combined.inverse();
  // Eigen::Affine3d xy_to_lm = lm_to_xy.inverse();
  // Eigen::Affine3d lm_combined_inv = map_to_pose * xy_to_lm;
  pcl::transformPointCloud(*merged_xy, *merged_map, lm_combined_inv);

  CloudPtr simple_merged_map(new Cloud());
  pcl::approximatePolygon2D<PointT>(
      merged_map->points, simple_merged_map->points, 0.005, false, true);

  Eigen::Vector4d map_lm_coeffs = planeCoefficients();
  for (std::size_t i = 0; i < merged_map->points.size(); i++) {
    // printf ("Merged Map: Boundary Point: %lf %lf %lf\n",
    // merged_map->points[i].x ,
    // merged_map->points[i].y ,
    // merged_map->points[i].z );

    double ptp_dist =
        fabs(map_lm_coeffs[0] * merged_map->points[i].x +
             map_lm_coeffs[1] * merged_map->points[i].y +
             map_lm_coeffs[2] * merged_map->points[i].z + map_lm_coeffs[3]);
    if (ptp_dist > 0.001) {
      LOG(INFO) << "ERROR: Merged Map: Point is " << ptp_dist << "from plane.";
      exit(1);
    }
  }

  // pcl::copyPointCloud(*simple_merged_map, *boundary_);
  pcl::copyPointCloud(*merged_map, *boundary_);
  // boundary_.swap(merged_map);
  //*boundary_ = *merged_map;
  // CloudPtr meas_boundary = plane.boundary();
  // CloudPtr map_boundary = boundary_;
}

template class omnimapper::BoundedPlane3<pcl::PointXYZRGBA>;
