#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper/omnimapper_visualizer_pcl.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
  // Create an omnimapper
  omnimapper::OmniMapperBase omb;

  // Make a timestamp
  boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
  gtsam::Symbol sym1;
  omb.GetPoseSymbolAtTime(t1, sym1);
  std::cout << "t1: " << t1 << " sym1: " << sym1.index() << std::endl;

  // Sleep and make another timestamp
  sleep(1);
  boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
  gtsam::Symbol sym2;
  omb.GetPoseSymbolAtTime(t2, sym2);
  std::cout << "t2: " << t2 << " sym2: " << sym2.index() << std::endl;
}
