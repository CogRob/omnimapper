#include <omnimapper/omnimapper_base.h>
#include <omnimapper/plugins/icp_plugin.h>
#include <omnimapper/omnimapper_visualizer_pcl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/common/time.h>
#include <boost/filesystem.hpp>
#include <pcl/segmentation/planar_region.h>
#include <omnimapper/plugins/plane_plugin.h>
#include <omnimapper/plugins/no_motion_pose_plugin.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;

// void
// testRotationOfHulls ()
// {
//   pcl::PointCloud<PointT> boundary;
//   PointT p1;
//   p1.x = 1.0;
//   p1.y = 1.0;
//   p1.z = 1.0;
//   boundary.points.push_back (p1);
//   PointT p2;
//   p2.x = 1.0;
//   p2.y = -1.0;
//   p2.z = 1.0;
//   boundary.points.push_back (p2);
//   PointT p3;
//   p3.x = -1.0;
//   p3.y = -1.0;
//   p3.z = 1.0;
//   boundary.points.push_back (p3);
//   PointT p4;
//   p4.x = -1.0;
//   p4.y = 1.0;
//   p4.z = 1.0;
//   boundary.points.push_back (p4);

//   Eigen::Vector4f centroid (0.0, 0.0, 1.0, 0.0);
//   pcl::PointCloud<PointT> empty_inliers;
//   std_msgs::Header empty_header;
//   // Make a plane
//   gtsam::Plane<Point> plane(0.0, 0.0, -1.0, 1.0, boundary, empty_inliers, centroid, empty_header);

//   pcl::PointCloud<PointT> initial_hull_ = plane.hull ();
// }

void
testPlanarTransform ()
{
  // Make a Plane
  pcl::PointCloud<PointT> boundary1;
  
  PointT p1;
  p1.x = 1.0;
  p1.y = 1.0;
  p1.z = 1.0;
  boundary1.points.push_back (p1);
  PointT p2;
  p2.x = 1.0;
  p2.y = -1.0;
  p2.z = 1.0;
  boundary1.points.push_back (p2);
  PointT p3;
  p3.x = -1.0;
  p3.y = -1.0;
  p3.z = 1.0;
  boundary1.points.push_back (p3);
  PointT p4;
  p4.x = -1.0;
  p4.y = 1.0;
  p4.z = 1.0;
  boundary1.points.push_back (p4);
  
  // Get the Coeffs
  Eigen::Vector4f centroid;
  Eigen::Matrix3f cov;
  pcl::computeMeanAndCovarianceMatrix (boundary1, cov, centroid);
  Eigen::Vector4f plane_params;

  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (cov, eigen_value, eigen_vector);
  plane_params[0] = eigen_vector[0];
  plane_params[1] = eigen_vector[1];
  plane_params[2] = eigen_vector[2];
  plane_params[3] = 0;
  plane_params[3] = -1 * plane_params.dot (centroid);

  Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
  vp[3] = 1.0;
  float cos_theta = vp.dot (plane_params);
  if (cos_theta < 0)
  {
    plane_params *= -1;
    plane_params[3] = 0;
    plane_params[3] = -1 * plane_params.dot (centroid);
  }
  
  printf ("Init Plane: %lf %lf %lf %lf\n", plane_params[0], plane_params[1], plane_params[2], plane_params[3]);
  printf ("Init Points:\n");
  //for (int i = 0; i < boundary1.points.size (); i++)
  //{
  //}

  // New Norm
  Eigen::Vector4f new_norm (-1.0, 0.0, 0.0, 2.0);
  
  Eigen::Vector3d prev_norm3 (0.0, 0.0, -1.0);
  Eigen::Vector3d new_norm3 (-1.0, 0.0, 0.0);
  double angle = acos (prev_norm3.dot (new_norm3));
  Eigen::Vector3d axis = prev_norm3.cross (new_norm3);
  Eigen::Affine3d transform;
  transform = Eigen::AngleAxisd (angle, axis);
  Eigen::Vector3d translation_part = new_norm3 * -1 * (new_norm[3] - plane_params[3]);
  transform.translation () = translation_part;
  std::cout << "Transform: " << transform.translation () << std::endl;
  pcl::PointCloud<PointT> transformed_boundary1;
  pcl::transformPointCloud (boundary1, transformed_boundary1, transform);
  for (int i = 0; i < transformed_boundary1.points.size (); i++)
  {
    printf ("%d: %lf %lf %lf\n", i, transformed_boundary1.points[i].x, transformed_boundary1.points[i].y, transformed_boundary1.points[i].z);
  }

}

void
testPlanarTransform2 ()
{
  pcl::visualization::PCLVisualizer viewer_ ("Viewer");
  viewer_.setBackgroundColor (0, 0, 0);
  viewer_.addCoordinateSystem (1.0, 0);
  viewer_.initCameraParameters ();

  // Make a Plane
  pcl::PointCloud<PointT> boundary1;
  
  PointT p1;
  p1.x = 1.0;
  p1.y = 1.0;
  p1.z = 1.0;
  boundary1.points.push_back (p1);
  PointT p2;
  p2.x = 1.0;
  p2.y = -1.0;
  p2.z = 1.0;
  boundary1.points.push_back (p2);
  PointT p3;
  p3.x = -1.0;
  p3.y = -1.0;
  p3.z = 1.0;
  boundary1.points.push_back (p3);
  PointT p4;
  p4.x = -1.0;
  p4.y = 1.0;
  p4.z = 1.0;
  boundary1.points.push_back (p4);
  
  char spherename[1024];
  for (int i = 0; i < boundary1.points.size (); i++)
  {
    sprintf (spherename, "orig_sphere_%d", i);
    boundary1.points[i].r = 255;
    boundary1.points[i].g = 0;
    boundary1.points[i].b = 0;
    viewer_.addSphere (boundary1.points[i], 0.05, 1.0, 0.0, 0.0, spherename);
  }

  //viewer_.addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(boundary1), "boundary1");

  // Get the Coeffs
  Eigen::Vector4f centroid;
  Eigen::Matrix3f cov;
  pcl::computeMeanAndCovarianceMatrix (boundary1, cov, centroid);
  Eigen::Vector4f plane_params;

  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
  pcl::eigen33 (cov, eigen_value, eigen_vector);
  plane_params[0] = eigen_vector[0];
  plane_params[1] = eigen_vector[1];
  plane_params[2] = eigen_vector[2];
  plane_params[3] = 0;
  plane_params[3] = -1 * plane_params.dot (centroid);

  Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
  vp[3] = 1.0;
  float cos_theta = vp.dot (plane_params);
  if (cos_theta < 0)
  {
    plane_params *= -1;
    plane_params[3] = 0;
    plane_params[3] = -1 * plane_params.dot (centroid);
  }
  
  printf ("Init Plane: %lf %lf %lf %lf\n", plane_params[0], plane_params[1], plane_params[2], plane_params[3]);
  printf ("Init Points:\n");
  // for (int i = 0; i < boundary1.points.size (); i++)
  // {
    
  // }

  // Create a transform
  gtsam::Rot3 sam_rot = gtsam::Rot3::ypr (0.0, 0.0, 0.785398163);
  gtsam::Point3 sam_trans (0.0, 0.0, 0.0);
  gtsam::Pose3 sam_pose (sam_rot, sam_trans);
  std::cout << sam_rot.matrix () << std::endl;
  std::cout << sam_trans.vector () << std::endl;

  Eigen::Affine3f true_transform = pose3ToTransform (sam_pose);
  std::cout << true_transform.rotation () << std::endl;
  std::cout << true_transform.translation () << std::endl;
  

  // Apply this transform to the points
  Eigen::Vector4f centroid1;
  Eigen::Matrix3f cov1;
  EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value1;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_vector1;
  pcl::PointCloud<PointT> true_transformed_boundary1;
  pcl::transformPointCloud (boundary1, true_transformed_boundary1, true_transform);
  pcl::computeMeanAndCovarianceMatrix (true_transformed_boundary1, cov1, centroid1);
  printf ("centroid: %lf %lf %lf\n", centroid1[0], centroid1[1], centroid1[2]);
  pcl::eigen33 (cov1, eigen_value1, eigen_vector1);
  Eigen::Vector4f true_transformed_plane_params;

  true_transformed_plane_params[0] = eigen_vector1[0];
  true_transformed_plane_params[1] = eigen_vector1[1];
  true_transformed_plane_params[2] = eigen_vector1[2];
  true_transformed_plane_params[3] = 0;
  true_transformed_plane_params[3] = -1 * true_transformed_plane_params.dot (centroid1);

  //Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
  //vp[3] = 1.0;
  float cos_theta1 = vp.dot (true_transformed_plane_params);
  if (cos_theta1 < 0)
  {
    true_transformed_plane_params *= -1;
    true_transformed_plane_params[3] = 0;
    true_transformed_plane_params[3] = -1 * true_transformed_plane_params.dot (centroid1);
  }

  printf ("True Transformed Plane: %lf %lf %lf %lf\n", true_transformed_plane_params[0], true_transformed_plane_params[1], true_transformed_plane_params[2], true_transformed_plane_params[3]);

  for (int i = 0; i < true_transformed_boundary1.points.size (); i++)
  {
    printf ("%d: %lf %lf %lf\n", i, true_transformed_boundary1.points[i].x, true_transformed_boundary1.points[i].y, true_transformed_boundary1.points[i].z);
  }

  for (int i = 0; i < true_transformed_boundary1.points.size (); i++)
  {
    sprintf (spherename, "true_sphere_%d", i);
    viewer_.addSphere (true_transformed_boundary1.points[i], 0.05, 0.0, 1.0, 0.0, spherename);
  }

  //viewer_.addPointCloud (boost::make_shared<pcl::PointCloud<PointT> >(true_transformed_boundary1), "true_boundary1");
  
  // Now compute the transform again, using the usual way
  Eigen::Vector3d orig_norm (plane_params[0], plane_params[1], plane_params[2]);
  Eigen::Vector3d new_norm (true_transformed_plane_params[0], true_transformed_plane_params[1], true_transformed_plane_params[2]);
  double angle = acos (orig_norm.dot (new_norm));
  Eigen::Vector3d axis = orig_norm.cross (new_norm);
  axis.normalize ();
  Eigen::Affine3d new_transform;
  new_transform = Eigen::AngleAxisd (angle, axis);
  pcl::PointCloud<PointT> our_transformed_boundary1;
  pcl::transformPointCloud (boundary1, our_transformed_boundary1, new_transform);

  std::cout << new_transform.rotation () << std::endl;
  std::cout << new_transform.translation () << std::endl;

  pcl::computeMeanAndCovarianceMatrix (our_transformed_boundary1, cov, centroid);
  pcl::eigen33 (cov, eigen_value, eigen_vector);
  Eigen::Vector4f our_transformed_plane_params;

  our_transformed_plane_params[0] = eigen_vector[0];
  our_transformed_plane_params[1] = eigen_vector[1];
  our_transformed_plane_params[2] = eigen_vector[2];
  our_transformed_plane_params[3] = 0;
  our_transformed_plane_params[3] = -1 * our_transformed_plane_params.dot (centroid);

  //Eigen::Vector4f vp = Eigen::Vector4f::Zero ();
  //vp[3] = 1.0;
  cos_theta = vp.dot (our_transformed_plane_params);
  if (cos_theta < 0)
  {
    our_transformed_plane_params *= -1;
    our_transformed_plane_params[3] = 0;
    our_transformed_plane_params[3] = -1 * our_transformed_plane_params.dot (centroid);
  }

  printf ("Our Transformed Plane: %lf %lf %lf %lf\n", our_transformed_plane_params[0], our_transformed_plane_params[1], our_transformed_plane_params[2], our_transformed_plane_params[3]);

  for (int i = 0; i < our_transformed_boundary1.points.size (); i++)
  {
    printf ("%d: %lf %lf %lf\n", i, our_transformed_boundary1.points[i].x, our_transformed_boundary1.points[i].y, our_transformed_boundary1.points[i].z);
  }

  for (int i = 0; i < our_transformed_boundary1.points.size (); i++)
  {
    sprintf (spherename, "our_sphere_%d", i);
    viewer_.addSphere (our_transformed_boundary1.points[i], 0.05, 0.0, 0.0, 1.0, spherename);
  }
  
  viewer_.spin ();
}



int main (int argc, char** argv)
{
  // Create an omnimapper
  omnimapper::OmniMapperBase omb;

  testPlanarTransform2 ();
  //testRotationOfHulls ();

  // Create a no motoin plane plugin
  // omnimapper::NoMotionPosePlugin no_motion_plugin (&omb);
  // boost::shared_ptr<omnimapper::PosePlugin> no_motion_ptr (&no_motion_plugin);
  // omb.addPosePlugin (no_motion_ptr);
  // sleep (1);
/*
  // Create a plane plugin
  omnimapper::PlaneMeasurementPlugin<PointT> plane_plugin (&omb);
  plane_plugin.setOverwriteTimestamps (false);
  //boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&, omnimapper::Time&)> plane_cb = boost::bind (&omnimapper::PlaneMeasurementPlugin<PointT>::planarRegionCallback, &plane_plugin, _1, _2);  

  // Create some fake planes
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > t1_regions;
  std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > t2_regions;  

  // 1m away, facing camera
  Eigen::Vector3f p1_centroid (0.0, 0.0, 1.0);
  Eigen::Matrix3f p1_cov = Eigen::Matrix3f::Identity ();
  int p1_count = 100;
  pcl::PointCloud<pcl::PointXYZRGBA> p1_border;
  p1_border.points.push_back (pcl::PointXYZRGBA (1.0,1.0,1.0));
  p1_border.points.push_back (pcl::PointXYZRGBA (1.0,-1.0,1.0));
  p1_border.points.push_back (pcl::PointXYZRGBA (-1.0,-1.0,1.0));
  p1_border.points.push_back (pcl::PointXYZRGBA (-1.0,1.0,1.0));
  Eigen::Vector4f p1_coeffs (0.0, 0.0, -1.0, 1.0);
  pcl::PlanarRegion<PointT> p1_region (p1_centroid, p1_cov, p1_count, p1_border.points, p1_coeffs);
  t1_regions.push_back (p1_region);

  // 1m away, facing camera, with error
  Eigen::Vector3f p2_centroid (0.0, 0.0, 0.9);
  Eigen::Matrix3f p2_cov = Eigen::Matrix3f::Identity ();
  int p2_count = 100;
  pcl::PointCloud<pcl::PointXYZ> p2_border;
  p2_border.points.push_back (pcl::PointXYZ (1.0,1.0,0.9));
  p2_border.points.push_back (pcl::PointXYZ (1.0,-1.0,0.9));
  p2_border.points.push_back (pcl::PointXYZ (-1.0,-1.0,0.9));
  p2_border.points.push_back (pcl::PointXYZ (-1.0,1.0,0.9));
  Eigen::Vector4f p2_coeffs (0.0, 0.0, -1.0, 0.9);
  pcl::PlanarRegion<PointT> p2_region (p2_centroid, p2_cov, p2_count, p2_border.points, p2_coeffs);
  t2_regions.push_back (p2_region);

  // Make a timestamp
  boost::posix_time::ptime t1 ( boost::posix_time::microsec_clock::local_time() );
  //gtsam::Symbol sym1;
  //omb.getPoseSymbolAtTime (t1, sym1);
  //std::cout << "t1: " << t1 << " sym1: " << sym1.index () << std::endl;
  
  // Sleep and make another timestamp
  sleep (1);
  boost::posix_time::ptime t2 ( boost::posix_time::microsec_clock::local_time() );
  //gtsam::Symbol sym2;
  //omb.getPoseSymbolAtTime (t2, sym2);
  //std::cout << "t2: " << t2 << " sym2: " << sym2.index () << std::endl;
  
  // Create some poses for these times
  gtsam::Symbol sym1;
  gtsam::Symbol sym2;
  omb.getPoseSymbolAtTime (t1, sym1);
  omb.getPoseSymbolAtTime (t2, sym2);
  Eigen::Matrix4f cloud_tform = Eigen::Matrix4f::Identity ();
  gtsam::Point3 trans (0.0, 0.0, -1.0);
  // gtsam::Pose3 relative_pose (gtsam::Rot3 (cloud_tform.block (0, 0, 3, 3).cast<double>()), 
  //                             gtsam::Point3 (cloud_tform (0,3), cloud_tform (1,3), cloud_tform (2,3)));
  gtsam::Pose3 relative_pose (gtsam::Rot3 (cloud_tform.block (0, 0, 3, 3).cast<double>()), trans);
  double trans_noise = 100.0;
  double rot_noise = 100.0;
  gtsam::SharedDiagonal noise = gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, rot_noise, rot_noise, rot_noise, trans_noise, trans_noise, trans_noise));
  gtsam::NonlinearFactor::shared_ptr between1 (new gtsam::BetweenFactor<gtsam::Pose3> (sym1, sym2, relative_pose, noise));
  omb.addFactor (between1);
  
  // Add the same measurements for both
  plane_plugin.planarRegionCallback (t1_regions, t1);
  omb.spinOnce ();

  sleep (2);
  omb.spinOnce ();
  
  // Get the latest solution
  gtsam::Values before = omb.getSolution ();
  before.print ("Before: ");

  // Add same measurement again
  plane_plugin.planarRegionCallback (t2_regions, t2);
  omb.spinOnce ();
  
  gtsam::Values after = omb.getSolution ();
  after.print ("After: ");
*/
  exit (0);
}
