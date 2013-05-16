#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/tf_pose_plugin.h>
#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper_ros/omnimapper_visualizer_rviz.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

typedef pcl::PointXYZ PointT;
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

    // Visualization
    omnimapper::OmniMapperVisualizerRViz<PointT> vis_plugin_;

    // Subscribers
    ros::Subscriber pointcloud_sub_;

    OmniMapperHandheldNode ()
      : n_ ("~"),
        omb_ (),
        tf_plugin_ (&omb_),
        icp_plugin_ (&omb_),
        vis_plugin_ (&omb_)
    {
      // Add the TF Pose Plugin
      boost::shared_ptr<omnimapper::PosePlugin> tf_plugin_ptr (&tf_plugin_);
      omb_.addPosePlugin (tf_plugin_ptr);

      // Set up an ICP Plugin
      icp_plugin_.setUseGICP (true);
      icp_plugin_.setOverwriteTimestamps (false);
      icp_plugin_.setMaxCorrespondenceDistance (0.1);
      icp_plugin_.setTransNoise (1000000.0);
      icp_plugin_.setRotNoise (1000000.0);
      

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
      CloudPtr cloud (new Cloud ());
      pcl::fromROSMsg (*msg, *cloud);
      icp_plugin_.cloudCallback (cloud);
    }
    
    
};

int
main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSHandheld");
  OmniMapperHandheldNode<PointT> omnimapper;
  ros::spin ();
}
