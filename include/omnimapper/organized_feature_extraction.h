// Boost
#include <boost/thread/thread.hpp>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/common/time.h>



// Useful macros
#define FPS_CALC(_WHAT_) \
do \
{ \
    static unsigned count = 0;\
    static double last = pcl::getTime ();\
    double now = pcl::getTime (); \
    ++count; \
    if (now - last >= 1.0) \
    { \
      std::cout << "Average framerate("<< _WHAT_ << "): " << double(count)/double(now - last) << " Hz" <<  std::endl; \
      count = 0; \
      last = now; \
    } \
}while(false)

//typedef pcl::PointXYZRGBA PointT;

namespace omnimapper
{

/** \brief OrganizedFeatureExtraction is our helper class for handling multiple types of feature extraction.
  *
  * \author Alex Trevor
  */
template <typename PointT>
class OrganizedFeatureExtraction
{
  typedef pcl::PointCloud<PointT> Cloud;
  typedef typename Cloud::Ptr CloudPtr;
  typedef typename Cloud::ConstPtr CloudConstPtr;
  typedef pcl::PointCloud<pcl::Normal> NormalCloud;
  typedef typename NormalCloud::Ptr NormalCloudPtr;
  typedef typename NormalCloud::ConstPtr NormalCloudConstPtr;

  protected:
    // PCL Grabber
    pcl::Grabber& grabber_;

    // Most recent cloud from the sensor
    CloudConstPtr prev_sensor_cloud_;
    boost::mutex sensor_cloud_mutex;

    // We process normals for frame n-1 concurrently with feature extraction for cloud n
    CloudConstPtr stage1_cloud_;
    NormalCloudPtr stage1_normals_;
    CloudConstPtr stage2_cloud_;
    NormalCloudConstPtr stage2_normals_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > stage3_regions_;
    CloudPtr stage3_occluding_cloud_;
    boost::mutex cloud_mutex;

    // Most recently processed cloud
    CloudConstPtr vis_cloud_;
    CloudConstPtr vis_occluding_cloud_;
    NormalCloudConstPtr vis_normals_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > vis_regions_;    
    boost::mutex vis_mutex;
    bool updated_data_;
    
    // Normal Estimation
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;

    // Plane Segmentation
    pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;

    // Edge Detection
    pcl::OrganizedEdgeFromRGBNormals<PointT, pcl::Normal, pcl::Label> oed;

    // Planar Region Callback
    boost::function<void(std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)> planar_region_callback_;

    // Edge Callbacks
    boost::function<void(const CloudConstPtr&)> occluding_edge_callback_;

    // Threads
    boost::thread vis_thread;
    boost::thread process_thread;

    // Flags
    bool debug_;

  public:
    OrganizedFeatureExtraction (pcl::Grabber& grabber);
    
    void cloudCallback (const CloudConstPtr& cloud);
    void processFrame ();
    void computeNormals ();
    void computePlanes ();
    void computeEdges ();
    void spin ();
    void setPlanarRegionCallback (boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >&)>& fn);
    void setOccludingEdgeCallback (boost::function<void (const CloudConstPtr&)>& fn);
    
};

}
