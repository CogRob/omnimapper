#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/omnimapper_visualizer_pcl.h>
//#include <omnimapper2_ros/rviz_output_plugin.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZ PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

int main(int argc, char** argv) {
  // Set up a PCDGrabber for some PCD files
  std::vector<std::string> pcd_files;
  boost::filesystem::directory_iterator end_itr;

  for (boost::filesystem::directory_iterator itr(argv[1]); itr != end_itr;
       ++itr) {
    if (itr->path().extension() == ".pcd") {
      pcd_files.push_back(itr->path().string());
    }
  }
  sort(pcd_files.begin(), pcd_files.end());
  printf("Found %d PCDs.\n", pcd_files.size());

  // Create a PCD Grabber
  pcl::PCDGrabber<PointT> grabber(pcd_files, 1.0, false);

  // Create an OmniMapper instance
  omnimapper::OmniMapperBase omb;
  omb.setDebug(true);

  // Start the OmniMapper thread
  boost::thread omb_thread(&omnimapper::OmniMapperBase::spin, &omb);

  // Create an ICP pose measurement plugin
  // omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin(&omb, grabber);
  omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin(&omb);
  icp_plugin.setUseGICP(true);
  icp_plugin.setMaxCorrespondenceDistance(3.5);
  icp_plugin.setScoreThreshold(1000.0);
  boost::function<void(const CloudConstPtr&)> f =
      boost::bind(&omnimapper::ICPPoseMeasurementPlugin<PointT>::cloudCallback,
                  &icp_plugin, _1);
  boost::signals2::connection c = grabber.registerCallback(f);
  grabber.start();

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
  boost::thread icp_thread(&omnimapper::ICPPoseMeasurementPlugin<PointT>::spin,
                           &icp_plugin);

  // while (grabber.isRunning ())
  while (true) {
    vis_pcl.spinOnce();
  }
  icp_thread.join();
  omb_thread.join();

  return 0;
}
