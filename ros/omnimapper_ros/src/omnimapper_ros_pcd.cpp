#include <omnimapper_ros/omnimapper_ros.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef typename pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;

int main(int argc, char** argv) {
  ros::init(argc, argv, "OmniMapperROSPCD");
  ros::NodeHandle nh("~");
  OmniMapperROS<pcl::PointXYZRGBA> omnimapper(nh);

  std::string pcd_dir;
  // nh.param ("pcd_dir", pcd_dir,

  // Start ROS Spinning
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Load PCDs
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

  //  Start calling cloud CB when ready
  bool done = false;
  int file_idx = 0;

  while (!done) {
    bool ready = true;

    // TODO: Check if mapper is ready
    boost::this_thread::sleep(boost::posix_time::seconds(1));

    // If we're done
    if (ready && (file_idx == static_cast<int>(pcd_files.size()))) {
      done = true;
      ROS_INFO("OmniMapperROS PCD: All files processed.");
      return 0;
    } else if (ready && (file_idx < static_cast<int>(pcd_files.size()))) {
      // Process a new frame
      CloudPtr cloud(new Cloud());
      pcl::io::loadPCDFile(pcd_files[file_idx], *cloud);

      sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2());
      pcl::toROSMsg(*cloud, *cloud_msg);
      file_idx++;
      omnimapper.cloudCallback(cloud_msg);
    } else {
      // Not ready
    }
  }
}
