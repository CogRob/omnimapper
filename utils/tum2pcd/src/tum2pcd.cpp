#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

#include <opencv2/opencv.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

float focalLength = 525.0;
float centerX = 319.5;
float centerY = 239.5;
float scalingFactor = 5000.0;

CloudPtr createXYZRGBPointCloud (std::string & rgb_filename, std::string & depth_filename, std::string & stamp)
{
  cv::Mat rgb = cv::imread (rgb_filename, -1); // load as-is
  cv::Mat depth = cv::imread (depth_filename, -1); // load as-is
  
  // Parse out the timestamp
  // Format is seconds.microseconds since epoch
  std::vector<std::string> strs;
  boost::split (strs, stamp, boost::is_any_of ("."));
  long seconds = boost::lexical_cast<long>(strs[0]);
  long microseconds = boost::lexical_cast<long>(strs[1]);

  //boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
  //boost::posix_time::ptime cloud_time = time_t_epoch + boost::posix_time::seconds (seconds) + boost::posix_time::microseconds (microseconds);
  boost::posix_time::time_duration since_epoch = boost::posix_time::seconds (seconds) + boost::posix_time::microseconds (microseconds);

  CloudPtr cloud (new Cloud);

  cloud->header.stamp = since_epoch.total_microseconds ();  
  cloud->is_dense = false;
  cloud->height = depth.rows;
  cloud->width = depth.cols;

  std::cout << "Processing cloud from: " << stamp << " with stamp: " << cloud->header.stamp << std::endl;

  cloud->points.resize (cloud->height * cloud->width);

  int width = cloud->width;
  int height = cloud->height;

  for (int r=0; r<cloud->height; r++)
  {
    for (int c=0; c<cloud->width; c++)
    {
      int idx = r*cloud->width+c;
      cloud->points[idx].b = rgb.at<cv::Vec3b> (r, c) [0];
      cloud->points[idx].g = rgb.at<cv::Vec3b> (r, c) [1];
      cloud->points[idx].r = rgb.at<cv::Vec3b> (r, c) [2];
            
      if (depth.at<unsigned short> (r, c) == 0) // nan point
      {
        cloud->points[idx].x = std::numeric_limits<float>::quiet_NaN ();
        cloud->points[idx].y = std::numeric_limits<float>::quiet_NaN ();
        cloud->points[idx].z = std::numeric_limits<float>::quiet_NaN ();
        continue;
      }
      float Z = static_cast<float> (depth.at<unsigned short> (r, c)) / scalingFactor;
      cloud->points[idx].x = (static_cast<float>(c) - centerX) * Z / focalLength;
      cloud->points[idx].y = (static_cast<float>(r) - centerY) * Z / focalLength;
      cloud->points[idx].z = Z;
    }
  }

  return cloud;
}

int main (int argc, char** argv)
{
  bool help = false;
  pcl::console::parse_argument (argc, argv, "-h", help);
  
  if (argc < 3 || help)
  {
    std::cout << argv[0] << " associate.txt output_path" << std::endl;
    std::cout << "(e.g.) " << argv[0] << " associate.txt pose.txt time.txt" << std::endl;
    return (-1);
  }

  std::vector<std::string> stamps;
  std::vector<std::string> rgb_files;
  std::vector<std::string> depth_files;
  
  std::ifstream associated_file (argv[1]);
  while (!associated_file.eof ())
  {
    char buf[512];
    associated_file.getline (buf, 512);
    
    // Skip comments
    if (buf[0] == '#')
      continue;
      
    if (!buf[0])
      continue;
      
    std::vector<std::string> strs;
    std::string str (buf);
    boost::split (strs, str, boost::is_any_of ("\n "));
    //stamps.push_back (strs[0]); // rgb time stamp
    stamps.push_back (strs[2]); // depth time stamp
    rgb_files.push_back (strs[1]);
    depth_files.push_back (strs[3]);
  }
  
  assert (stamps.size () == rgb_files.size () && rgb_files.size () == depth_files.size ());
  size_t numof_total_frames = depth_files.size ();
  for (size_t i=0; i<numof_total_frames; i++)
  {
    //std::cout << stamps[i] << " " << pcd_files[i] << std::endl;
    //std::cout << stamps[i] << " " << rgb_files[i] << " " << depth_files[i] << std::endl;
  }

  std::vector<double> time_list;
  for (int i = 0; i < numof_total_frames; i++)
  {
    CloudPtr cloud_ptr = createXYZRGBPointCloud (rgb_files[i], depth_files[i], stamps[i]);
   
    std::string outfile (argv[2]);
    outfile.append (rgb_files[i]);
    outfile.append (std::string (".pcd"));
    
    //std::cout << "Would write: " << outfile << std::endl;
    
    
    //pcl::io::savePCDFileBinaryCompressed (outfile, *cloud_ptr);
    pcl::io::savePCDFile (outfile, *cloud_ptr);
    // boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time ();
    // eval->process (stamps[i], cloud_ptr);
    // boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time ();
    // double time = static_cast<double>((current_time - start_time).total_nanoseconds ()) * 1e-6;
    // std::cout << "[" << i+1 << "/" << numof_total_frames << "] in processing... takes " << time << " ms..." << std::endl;
    // time_list.push_back (time);
  }

  return 0;
}

