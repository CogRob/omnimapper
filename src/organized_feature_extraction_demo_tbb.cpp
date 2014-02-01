#include <omnimapper/organized_feature_extraction_tbb.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/image_viewer.h>
#include <X11/Xlib.h>
#include <google/profiler.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef typename Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointCloud<pcl::Label> LabelCloud;
typedef typename LabelCloud::Ptr LabelCloudPtr;
typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

template <typename PointT>
class OrganizedFeatureExtractionDemoTBB
{
  public:
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;
    typedef pcl::PointCloud<pcl::Label> LabelCloud;
    typedef typename LabelCloud::Ptr LabelCloudPtr;
    typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

  protected:
    boost::shared_ptr<pcl::visualization::ImageViewer> image_viewer_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    
    bool updated_;
    CloudConstPtr prev_cloud_;
    LabelCloudConstPtr prev_labels_;
    std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > prev_regions_;
    boost::mutex cloud_mutex_;

  public:
    OrganizedFeatureExtractionDemoTBB ()
      : image_viewer_ (new pcl::visualization::ImageViewer ("Image Viewer")),
        viewer_ (new pcl::visualization::PCLVisualizer ("3D Viewer")),
        updated_ (false),
        prev_cloud_ (new Cloud ()),
        prev_labels_ (new LabelCloud ())
    {
      printf ("Starting demo.\n");
    }
    
    void
    clusterLabelsCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
    {
      boost::lock_guard<boost::mutex> lock (cloud_mutex_);
      printf ("cluster labels_cb: %d %d\n", cloud->points.size (), labels->points.size ());
      prev_cloud_ = cloud;
      prev_labels_ = labels;
      updated_ = true;
      FPS_CALC ("demo callback");
    }
    
    void
    spinVis ()
    {
      CloudConstPtr cloud (new Cloud ());
      LabelCloudConstPtr labels (new LabelCloud ());
      bool should_update = false;
      // Get new data, if there is any
      if (cloud_mutex_.try_lock ()){
        if (updated_)
        {
          prev_cloud_.swap (cloud);
          prev_labels_.swap (labels);
          updated_ = false;
          printf ("cloud points: %d labels points: %d\n", cloud->points.size (), labels->points.size ());
          if (cloud->points.size () > 200 && labels->points.size () > 200)
          {
            should_update = true;
          }
        }
        cloud_mutex_.unlock ();
      }
      
      // If we got new data, draw it
      if (should_update)
      {
        CloudPtr color_cloud (new Cloud (*cloud));
        unsigned char red [6] = {255,   0,   0, 255, 255,   0};
        unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
        unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};
        
        for (size_t i = 0; i < cloud->points.size (); i++)
        {
          // color_cloud->points[i].r = red[labels->points[i].label%6];
          // color_cloud->points[i].g = grn[labels->points[i].label%6];
          // color_cloud->points[i].b = blu[labels->points[i].label%6];
          
          if (labels->points[i].label == std::numeric_limits<unsigned>::max ())
          {
            // Do nothing
          }
          else
          {
            color_cloud->points[i].r = (cloud->points[i].r + red[labels->points[i].label%6]) / 2;
            color_cloud->points[i].g = (cloud->points[i].g + grn[labels->points[i].label%6]) / 2;
            color_cloud->points[i].b = (cloud->points[i].b + blu[labels->points[i].label%6]) / 2;
          }
          
        } 
        if (color_cloud->points.size () > 200)
          image_viewer_->addRGBImage<PointT>(color_cloud, "label_image", 0.2);

      }
      image_viewer_->spinOnce ();
      printf ("spinning in spinvis...\n");
      

    }
    
    
};

int
main (int argc, char** argv)
{
  XInitThreads ();

  // Create a grabber, and hook it up to the feature extraction
  pcl::OpenNIGrabber ni_grabber ("#1");
  omnimapper::OrganizedFeatureExtractionTBB<PointT> ofe;

  boost::function<void (const CloudConstPtr&)> f = boost::bind (&omnimapper::OrganizedFeatureExtractionTBB<PointT>::cloudCallback, &ofe, _1);
  boost::signals2::connection c = ni_grabber.registerCallback (f);

  OrganizedFeatureExtractionDemoTBB<PointT> demo;
  boost::function<void(const CloudConstPtr&, const LabelCloudConstPtr&)> label_callback = boost::bind (&OrganizedFeatureExtractionDemoTBB<PointT>::clusterLabelsCallback, &demo, _1, _2);
  //ofe.setPlaneLabelsCallback (label_callback);
  ofe.setClusterLabelsCallback (label_callback);

  ni_grabber.start ();

  ProfilerStart("tbb.prof");  
  while (true)
  {
    boost::this_thread::sleep (boost::posix_time::milliseconds (10));
    
    ofe.spinOnce ();
    
    demo.spinVis ();
  }
    ProfilerStop ();

  return (0);
}
