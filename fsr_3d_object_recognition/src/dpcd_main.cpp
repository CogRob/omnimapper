#include "tbb/task_group.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>


int describeDatabase (std::string database, bool append);
int createDescription (std::string view, std::string viewfolder, std::ofstream &dfile);
int getViewInfo(std::string fname, std::string &class_name, std::string &class_num, std::string &view_num);

namespace fs = boost::filesystem;

int main (int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile (argv[1], *cloud);

  for (size_t i = 0; i < cloud->points.size (); i++)
  {
    std::cout << cloud->points[i] << std::endl;
  }

  return 0;
}
