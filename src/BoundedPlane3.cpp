#include <omnimapper/BoundedPlane3.h>
#include <omnimapper/geometry.h>
#include <omnimapper/transform_helpers.h>
#include <omnimapper/impl/geometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

/* ************************************************************************* */
/// The print fuction
template <typename PointT> void 
omnimapper::BoundedPlane3<PointT>::print(const std::string& s) const {
  gtsam::Vector coeffs = planeCoefficients ();
  std::cout << s << " : " << coeffs << std::endl;
}

template <typename PointT> void
omnimapper::BoundedPlane3<PointT>::reprojectBoundary ()
{
  
}

template <typename PointT> omnimapper::BoundedPlane3<PointT> 
omnimapper::BoundedPlane3<PointT>::retract (const gtsam::Vector& v) const
{
  boost::lock_guard<boost::mutex> lock (*plane_mutex_);
  std::cout << "BoundedPlane3: retracting: " << v << std::endl;
  
  // Retract coefficients
  gtsam::Vector2 n_v (v (0), v (1));
  gtsam::Sphere2 n_retracted = n_.retract (n_v, gtsam::Sphere2::EXPMAP);
  double d_retracted = d_ + v (2);

  // Retract the boundary too.
  Eigen::Vector3d n_retracted_unit = n_retracted.unitVector ();
  Eigen::Vector4d new_coeffs (n_retracted_unit[0], n_retracted_unit[1], n_retracted_unit[2], d_retracted);
  Eigen::Vector4d old_coeffs = planeCoefficients ();
  //Eigen::Affine3d transform = planarAlignmentTransform (old_coeffs, new_coeffs);
  Eigen::Affine3d transform = planarAlignmentTransform (new_coeffs, old_coeffs);

  std::cout << "BoundedPlane3: transform translation: " << transform.translation () << std::endl;

  CloudPtr new_boundary (new Cloud ());
  pcl::transformPointCloud (*boundary_, *new_boundary, transform);
  //pcl::copyPointCloud(*new_boundary, *boundary_);
  //pcl::transformPointCloud(*boundary_, *boundary_, transform);
  //*boundary_ = *new_boundary;
  //boundary_.swap(new_boundary);

 //return (omnimapper::BoundedPlane3<PointT>(n_retracted, d_retracted, boundary_, plane_mutex_));
  return (omnimapper::BoundedPlane3<PointT>(n_retracted, d_retracted, new_boundary, plane_mutex_));
  //return (omnimapper::BoundedPlane3<PointT>(n_retracted, d_retracted, boundary_));
}

template <typename PointT> gtsam::Vector 
omnimapper::BoundedPlane3<PointT>::localCoordinates(const omnimapper::BoundedPlane3<PointT>& y) const {
  gtsam::Vector n_local = n_.localCoordinates (y.n_, gtsam::Sphere2::EXPMAP);
  double d_local = d_ - y.d_;
  return gtsam::Vector (3) << n_local (0), n_local (1), -d_local;
}

template <typename PointT> gtsam::Vector 
omnimapper::BoundedPlane3<PointT>::planeCoefficients () const 
{
  gtsam::Vector unit_vec = n_.unitVector ();
  return (gtsam::Vector (4) << unit_vec[0], unit_vec[1], unit_vec[2], d_);
}

template <typename PointT> omnimapper::BoundedPlane3<PointT>
omnimapper::BoundedPlane3<PointT>::Transform (const omnimapper::BoundedPlane3<PointT>& plane,
                                              const gtsam::Pose3& xr,
                                              boost::optional<gtsam::Matrix&> Hr,
                                              boost::optional<gtsam::Matrix&> Hp)
{
  // TODO: this should be renamed, as it does not transform the planar boundary, only the coefficients.
  gtsam::Matrix n_hr;
  gtsam::Matrix n_hp;
  gtsam::Sphere2 n_rotated = xr.rotation ().unrotate (plane.n_, n_hr, n_hp);
  
  gtsam::Vector n_unit = plane.n_.unitVector ();
  gtsam::Vector unit_vec = n_rotated.unitVector ();
  double pred_d = n_unit.dot (xr.translation ().vector ()) + plane.d_;
  //OrientedPlane3 transformed_plane (unit_vec (0), unit_vec (1), unit_vec (2), pred_d);

  CloudPtr transformed_boundary (new Cloud ());
  Eigen::Affine3f transform = pose3ToTransform (xr);
  //pcl::transformPointCloud (*(plane.boundary_), *transformed_boundary, transform);

  BoundedPlane3<PointT> transformed_plane (unit_vec (0), unit_vec (1), unit_vec (2), pred_d, plane.boundary_, plane.plane_mutex_);
  //BoundedPlane3<PointT> transformed_plane (unit_vec (0), unit_vec (1), unit_vec (2), pred_d, transformed_boundary);

  if (Hr)
  {
    *Hr = gtsam::zeros (3, 6);
    (*Hr).block<2,3> (0,0) = n_hr;
    (*Hr).block<1,3> (2,3) = unit_vec;
  }
  if (Hp)
  {
    gtsam::Vector xrp = xr.translation ().vector ();
    gtsam::Matrix n_basis = plane.n_.basis();
    gtsam::Vector hpp = n_basis.transpose() * xrp;
    *Hp = gtsam::zeros (3,3);
    (*Hp).block<2,2> (0,0) = n_hp;
    (*Hp).block<1,2> (2,0) = hpp;
    (*Hp) (2,2) = 1;
  }
  
  return (transformed_plane);
}

template <typename PointT> Eigen::Vector4d
omnimapper::BoundedPlane3<PointT>::TransformCoefficients (const omnimapper::BoundedPlane3<PointT>& plane, const gtsam::Pose3& xr)
{
  gtsam::Sphere2 n_rotated = xr.rotation ().unrotate (plane.n_);
  gtsam::Vector n_rotated_unit = n_rotated.unitVector ();
  gtsam::Vector n_unit = plane.n_.unitVector ();
  double pred_d = n_unit.dot (xr.translation ().vector ()) + plane.d_;
  Eigen::Vector4d result (n_rotated_unit[0], n_rotated_unit[1], n_rotated_unit[2], pred_d);

  // if (xr.translation ().vector ().dot (n_rotated_unit) > 0)
  // {
  //   result = -result;
  //   printf ("BoundedPlane3: flipping normal on transform coeffs!\n");
  // }

  return (result);
}

/* ************************************************************************* */
template <typename PointT> gtsam::Vector 
omnimapper::BoundedPlane3<PointT>::error (const omnimapper::BoundedPlane3<PointT>& plane) const
{
  gtsam::Vector n_error = -n_.localCoordinates (plane.n_, gtsam::Sphere2::EXPMAP);

  if (!(std::isfinite (n_error[0]) && std::isfinite (n_error[1])))
  {
    printf ("BoundedPlane3: ERROR: Got NaN error on local coords!\n");
    //exit (3);
  }

  double d_error = d_ - plane.d_;
  std::cout << "BoundedPlane3: error: " << n_error << " " << d_error << std::endl;
  return (gtsam::Vector (3) << n_error (0), n_error (1), d_error);
}

template <typename PointT> void 
omnimapper::BoundedPlane3<PointT>::extendBoundary (const gtsam::Pose3& pose, BoundedPlane3<PointT>& plane) const
{
  boost::lock_guard<boost::mutex> lock (*plane_mutex_);
  Eigen::Vector4d z_axis (0.0, 0.0, 1.0, 0.0);
  //return;
  //  Move map cloud to the pose
  Eigen::Affine3d map_to_pose = pose3ToTransform (pose).cast<double>();
  Eigen::Affine3d pose_to_map = map_to_pose.inverse ();
  Eigen::Vector4d lm_coeffs = TransformCoefficients(*this, pose);
  Eigen::Affine3d lm_to_xy = planarAlignmentTransform(z_axis, lm_coeffs);
  Eigen::Affine3d xy_to_lm = planarAlignmentTransform(lm_coeffs, z_axis);
  Eigen::Affine3d lm_combined = lm_to_xy * pose_to_map;//map_to_pose * lm_to_xy;
  Eigen::Affine3d lm_combined_inv = map_to_pose * xy_to_lm;
  CloudPtr map_xy (new Cloud());
  printf ("BoundedPlane3: transforming landmark to pose.\n");
  pcl::transformPointCloud (*boundary_, *map_xy, lm_combined);
  printf ("BoundedPlane3: transformed.\n");

  // Move the measurement to the xy plane
  Eigen::Vector4d meas_coeffs = plane.planeCoefficients();
  CloudPtr meas_boundary = plane.boundary();
  //Eigen::Affine3d meas_to_xy = planarAlignmentTransform(z_axis, meas_coeffs);
  Eigen::Affine3d meas_to_xy = planarAlignmentTransform(meas_coeffs, z_axis);
  CloudPtr meas_xy(new Cloud());
  printf("BoundedPlane3: transforming measurement to xy.\n");
  pcl::transformPointCloud(*meas_boundary, *meas_xy, meas_to_xy);
  printf("BoundedPlane3: transformed.  Merging.\n");

  // TEST
  for (int i = 0; i < boundary_->points.size (); i++)
  {
   printf ("Internal Boundary: Boundary Point: %lf %lf %lf\n", 
     meas_xy->points[i].x ,
     meas_xy->points[i].y ,
     meas_xy->points[i].z );

   double ptp_dist = fabs (z_axis[0] * meas_xy->points[i].x +
     z_axis[1] * meas_xy->points[i].y +
     z_axis[2] * meas_xy->points[i].z +
     z_axis[3]);
   printf ("MeasXY: Point is %lf from plane.\n", ptp_dist);
  }

  for (int i = 0; i < meas_xy->points.size (); i++)
  {
   printf ("MeasXY: Boundary Point: %lf %lf %lf\n", 
     meas_xy->points[i].x ,
     meas_xy->points[i].y ,
     meas_xy->points[i].z );

   double ptp_dist = fabs (z_axis[0] * meas_xy->points[i].x +
     z_axis[1] * meas_xy->points[i].y +
     z_axis[2] * meas_xy->points[i].z +
     z_axis[3]);
   printf ("MeasXY: Point is %lf from plane.\n", ptp_dist);
  }

 for (int i = 0; i < map_xy->points.size (); i++)
 {
   printf ("MapXY: Boundary Point: %lf %lf %lf\n", 
     map_xy->points[i].x ,
     map_xy->points[i].y ,
     map_xy->points[i].z );

   double ptp_dist = fabs (z_axis[0] * map_xy->points[i].x +
     z_axis[1] * map_xy->points[i].y +
     z_axis[2] * map_xy->points[i].z +
     z_axis[3]);
   printf ("MapXY: Point is %lf from plane.\n", ptp_dist);
 }

 bool boundary_intersects = boost::geometry::intersects(boundary_->points);
 if (boundary_intersects)
  printf("BoundedPlane3: map boundary intersects!\n");
 bool meas_boundary_intersects = boost::geometry::intersects(meas_boundary->points);
 if (meas_boundary_intersects)
  printf("BoundedPlane3: meas boundary intersects!\n");

  printf("BoundedPlane3: Attempting to merge meas_xy (%d) with map_xy (%d)\n", meas_xy->points.size(), map_xy->points.size());
  bool meas_xy_intersect = boost::geometry::intersects(meas_xy->points);
  if (meas_xy_intersect)
    printf("BoundedPlane3: Meas_xy_intersects!\n");
  bool map_xy_intersect = boost::geometry::intersects(map_xy->points);
  if (map_xy_intersect)
    printf("BoundedPlane3: Map_xy intersects!\n");
  // TEST

  // Fuse
  CloudPtr merged_xy(new Cloud());
  bool worked = omnimapper::fusePlanarPolygonsXY<PointT> (*meas_xy, *map_xy, *merged_xy);
  if (!worked)
  {
    printf("BoundedPlane3: Error inside extend!\n");
    exit(1);
  }
  printf("BoundedPlane3: Merged: map: %d meas: %d combined: %d \n", map_xy->points.size(), meas_xy->points.size(), merged_xy->points.size());

  // Now move it back to the map
  CloudPtr merged_map (new Cloud());
  //Eigen::Affine3d lm_combined_inv = lm_combined.inverse();
  //Eigen::Affine3d xy_to_lm = lm_to_xy.inverse();
  //Eigen::Affine3d lm_combined_inv = map_to_pose * xy_to_lm;
  pcl::transformPointCloud(*merged_xy, *merged_map, lm_combined_inv);

  Eigen::Vector4d map_lm_coeffs = planeCoefficients();
   for (int i = 0; i < merged_map->points.size (); i++)
   {
     printf ("Merged Map: Boundary Point: %lf %lf %lf\n", 
       merged_map->points[i].x ,
       merged_map->points[i].y ,
       merged_map->points[i].z );

     double ptp_dist = fabs (map_lm_coeffs[0] * merged_map->points[i].x +
                             map_lm_coeffs[1] * merged_map->points[i].y +
                             map_lm_coeffs[2] * merged_map->points[i].z +
                             map_lm_coeffs[3]);
     printf ("Merged Map: Point is %lf from plane.\n", ptp_dist);
   }

   pcl::copyPointCloud(*merged_map, *boundary_);
   //boundary_.swap(merged_map);
  //*boundary_ = *merged_map;
  //CloudPtr meas_boundary = plane.boundary();
  //CloudPtr map_boundary = boundary_;  
}

template class omnimapper::BoundedPlane3<pcl::PointXYZRGBA>;
