#include <omnimapper/omnimapper_base.h>
#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/omnimapper_visualizer_pcl.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/common/time.h>
#include <boost/filesystem.hpp>
#include <pcl/segmentation/planar_region.h>
#include <omnimapper/plane_plugin.h>
#include <omnimapper/no_motion_pose_plugin.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

typedef pcl::PointXYZ PointT;

int main (int argc, char** argv)
{
  // Create an omnimapper
  omnimapper::OmniMapperBase omb;

  // Create a no motoin plane plugin
  // omnimapper::NoMotionPosePlugin no_motion_plugin (&omb);
  // boost::shared_ptr<omnimapper::PosePlugin> no_motion_ptr (&no_motion_plugin);
  // omb.addPosePlugin (no_motion_ptr);
  // sleep (1);

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
  pcl::PointCloud<pcl::PointXYZ> p1_border;
  p1_border.points.push_back (pcl::PointXYZ (1.0,1.0,1.0));
  p1_border.points.push_back (pcl::PointXYZ (1.0,-1.0,1.0));
  p1_border.points.push_back (pcl::PointXYZ (-1.0,-1.0,1.0));
  p1_border.points.push_back (pcl::PointXYZ (-1.0,1.0,1.0));
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

  exit (0);
}
