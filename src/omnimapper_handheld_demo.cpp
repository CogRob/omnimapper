#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/no_motion_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/omnimapper_visualizer_pcl.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper/plane_plugin.h>
//#include <omnimapper2_ros/rviz_output_plugin.h>
#include <pcl/common/time.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

int main(int argc, char** argv) {
  // Set up an openni grabber
  pcl::OpenNIGrabber grabber("#1");
  // Give a fake to the ICP plugin..  TODO: Fix this!
  std::vector<std::string> empty_pcd_files;
  pcl::PCDGrabber<PointT> fake_grabber(empty_pcd_files, 0.5, false);

  // Load files, if a directory is provided
  std::vector<std::string> pcd_files;
  boost::filesystem::directory_iterator end_itr;

  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr;
       ++itr) {
    if (itr->path().extension() == ".pcd") {
      pcd_files.push_back(itr->path().string());
    }
  }
  sort(pcd_files.begin(), pcd_files.end());
  printf("Found %zu PCDs.\n", pcd_files.size());

  // Create a PCD Grabber
  pcl::PCDGrabber<PointT> file_grabber(pcd_files, 1.0, false);

  // Set up a Feature Extraction
  omnimapper::OrganizedFeatureExtraction<PointT> ofe(grabber);

  // Create an OmniMapper instance
  omnimapper::OmniMapperBase omb;

  // Create a no motion pose plugin, to add a weak prior of no movement
  // Also serves to keep the pose chain connected in the case that ICP fails
  omnimapper::NoMotionPosePlugin no_motion_plugin(&omb);
  boost::shared_ptr<omnimapper::PosePlugin> no_motion_ptr(&no_motion_plugin);
  omb.addPosePlugin(no_motion_ptr);

  // Create an ICP pose measurement plugin
  // omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin(&omb,
  // fake_grabber);
  omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin(&omb);
  icp_plugin.setMaxCorrespondenceDistance(0.15);
  icp_plugin.setShouldDownsample(false);
  icp_plugin.setUseGICP(false);
  boost::function<void(const CloudConstPtr&)> icp_cloud_cb =
      boost::bind(&omnimapper::ICPPoseMeasurementPlugin<PointT>::cloudCallback,
                  &icp_plugin, _1);
  // ofe.setOccludingEdgeCallback (icp_cloud_cb);

  // Create a plane plugin
  omnimapper::PlaneMeasurementPlugin<PointT> plane_plugin(&omb);
  boost::function<void(
      std::vector<pcl::PlanarRegion<PointT>,
                  Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
      omnimapper::Time)>
      plane_cb = boost::bind(
          &omnimapper::PlaneMeasurementPlugin<PointT>::planarRegionCallback,
          &plane_plugin, _1, _2);
  ofe.setPlanarRegionStampedCallback(plane_cb);

  // Create a visualizer
  omnimapper::OmniMapperVisualizerPCL<PointT> vis_pcl(&omb);
  vis_pcl.spinOnce();
  boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr(&vis_pcl);
  omb.addOutputPlugin(vis_ptr);

  // Set the ICP Plugin on the visualizer
  boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_ptr(
      &icp_plugin);
  vis_pcl.setICPPlugin(icp_ptr);

  // Start the ICP thread
  // boost::thread
  // icp_thread(&omnimapper::ICPPoseMeasurementPlugin<PointT>::spin,
  // &icp_plugin);

  // Start the Feature Extraction Thread
  boost::thread ofe_thread(
      &omnimapper::OrganizedFeatureExtraction<PointT>::spin, &ofe);

  // Start the OmniMapper thread
  boost::thread omb_thread(&omnimapper::OmniMapperBase::spin, &omb);

  // while (grabber.isRunning ())
  while (true) {
    vis_pcl.spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
  }
  // icp_thread.join ();
  ofe_thread.join();
  omb_thread.join();

  return 0;
}
