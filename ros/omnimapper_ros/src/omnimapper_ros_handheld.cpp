#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/tf_pose_plugin.h>
#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/plane_plugin.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper_ros/omnimapper_visualizer_rviz.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <pcl/io/pcd_grabber.h>
#include <boost/filesystem.hpp>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

template <typename PointT> 
class OmniMapperHandheldNode
{
  public:
    // ROS Node Handle
    ros::NodeHandle n_;
    
    // OmniMapper Instance
    omnimapper::OmniMapperBase omb_;

    // TF Pose Plugin
    omnimapper::TFPosePlugin tf_plugin_;

    // ICP Plugin
    omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin_;

    // Plane Plugin
    omnimapper::PlaneMeasurementPlugin<PointT> plane_plugin_;

    // Visualization
    omnimapper::OmniMapperVisualizerRViz<PointT> vis_plugin_;

    // Fake Grabber (TODO: Fix this)
    std::vector<std::string> empty_files_;
    pcl::PCDGrabber<PointT> fake_grabber_;
    // Organized Feature Extraction
    omnimapper::OrganizedFeatureExtraction<PointT> organized_feature_extraction_;

    // Subscribers
    ros::Subscriber pointcloud_sub_;

    OmniMapperHandheldNode ()
      : n_ ("~"),
        omb_ (),
        tf_plugin_ (&omb_),
        icp_plugin_ (&omb_),
        plane_plugin_ (&omb_),
        vis_plugin_ (&omb_),
        fake_grabber_ (empty_files_, 1.0, false),
        organized_feature_extraction_ (fake_grabber_)
    {
      // Add the TF Pose Plugin
      boost::shared_ptr<omnimapper::PosePlugin> tf_plugin_ptr (&tf_plugin_);
      omb_.addPosePlugin (tf_plugin_ptr);

      // Set up an ICP Plugin
      icp_plugin_.setUseGICP (true);
      icp_plugin_.setOverwriteTimestamps (false);
      icp_plugin_.setAddIdentityOnFailure (false);
      icp_plugin_.setShouldDownsample (true);
      icp_plugin_.setLeafSize (0.02);
      icp_plugin_.setMaxCorrespondenceDistance (0.5);
      icp_plugin_.setScoreThreshold (0.8);
      icp_plugin_.setTransNoise (1.1);//10.1
      icp_plugin_.setRotNoise (1.1);//10.1
      icp_plugin_.setAddLoopClosures (true);
      icp_plugin_.setLoopClosureDistanceThreshold (1.0);

      // Set up the Feature Extraction
      plane_plugin_.setOverwriteTimestamps (false);
      plane_plugin_.setDisableDataAssociation (false);
      plane_plugin_.setRangeThreshold (0.6);
      boost::function<void (std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >, omnimapper::Time)> plane_cb = boost::bind (&omnimapper::PlaneMeasurementPlugin<PointT>::planarRegionCallback, &plane_plugin_, _1, _2);
      organized_feature_extraction_.setPlanarRegionStampedCallback (plane_cb);

      // Set the ICP Plugin on the visualizer
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_ptr (&icp_plugin_);
      vis_plugin_.setICPPlugin (icp_ptr);
      
      // Subscribe to Point Clouds
      pointcloud_sub_ = n_.subscribe ("/throttled_points", 1, &OmniMapperHandheldNode::cloudCallback, this);//("/camera/depth/points", 1, &OmniMapperHandheldNode::cloudCallback, this);
      
      // Install the visualizer
      boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr (&vis_plugin_);
      omb_.addOutputPlugin (vis_ptr);

      //OmniMapper thread
      omb_.setDebug (true);
      boost::thread omb_thread (&omnimapper::OmniMapperBase::spin, &omb_); 
      boost::thread icp_thread(&omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &icp_plugin_);
    }

    void
    cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      ROS_INFO ("OmniMapperROS got a cloud.");
      //CloudPtr cloud (new Cloud ());
      //pcl::fromROSMsg (*msg, *cloud);
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg (*msg, *xyz_cloud);
      CloudPtr cloud (new Cloud ());
      pcl::copyPointCloud (*xyz_cloud, *cloud);
      icp_plugin_.cloudCallback (cloud);
      organized_feature_extraction_.cloudCallback (cloud);
    }
    
    
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSHandheld");
  OmniMapperHandheldNode<PointT> omnimapper;
  ros::spin ();
}
