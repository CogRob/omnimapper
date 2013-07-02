#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include <ros/sensor_msgs/laserscan.h>
#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/ros_time_utils.h>
#include <omnimapper_ros/tf_pose_plugin.h>
#include <omnimapper_ros_csm/canonical_scan_matcher_plugin.h>
#include <omnimapper_ros_csm/csm_visualizer.h>
//#include <omnimapper_ros/ros_tf_utils.h>

typedef boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr;
typedef sensor_msgs::LaserScan lscan;

//template<typename lscanPtr>
template<typename lscanT>
class OmniMapperPelicanNode
{
  public:
    // ROS Node Handle
    ros::NodeHandle n_;

    // OmniMapper Instance
    omnimapper::OmniMapperBase omb_;

    // TF Pose Plugin
    omnimapper::TFPosePlugin tf_plugin_;

    // ICP Plugin
    //omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin_;

    //CSM Plugin
    omnimapper::CanonicalScanMatcherPlugin<lscanT> csm_plugin_;

    // Visualization
    //omnimapper::OmniMapperVisualizerRViz<lscanT> vis_plugin_;
    omnimapper::CSMVisualizerRViz<sensor_msgs::LaserScan> vis_plugin_;

    // Subscribers
    ros::Subscriber laserScan_sub_;

    std::string odom_frame_name_;
    std::string base_frame_name_;

    OmniMapperPelicanNode ()
      : n_ ("~"),
        omb_ (),
        tf_plugin_ (&omb_),
        csm_plugin_ (&omb_),
        vis_plugin_ (&omb_)
        //vis_plugin_ (&omb_)
    {
      odom_frame_name_ = std::string ("/odom");
      base_frame_name_ = std::string ("/utm30");

      omb_.setSuppressCommitWindow(true);
      tf_plugin_.setTranslationNoise (0.1);
      //tf_plugin_.setRotationNoise (0.1);
      tf_plugin_.setRollNoise (0.0001);
      tf_plugin_.setPitchNoise (0.0001);
      tf_plugin_.setYawNoise (0.1);
      tf_plugin_.setOdomFrameName (odom_frame_name_);
      tf_plugin_.setBaseFrameName (base_frame_name_);

      // Add the CSM plugin to the visualizer
      boost::shared_ptr<omnimapper::CanonicalScanMatcherPlugin<lscanT> > csm_ptr (&csm_plugin_);
      vis_plugin_.setCSMPlugin (csm_ptr);

      // Add the TF Pose Plugin
      boost::shared_ptr<omnimapper::PosePlugin> tf_plugin_ptr (&tf_plugin_);
      omb_.addPosePlugin (tf_plugin_ptr);

      // Subscribe to laser scan
      laserScan_sub_ = n_.subscribe ("/scan", 1, &OmniMapperPelicanNode::laserScanCallback, this);

      // Install the visualizer
      boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr (&vis_plugin_);
      omb_.addOutputPlugin (vis_ptr);
      //boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr (&vis_plugin_);
      //omb_.addOutputPlugin (vis_ptr);

      //OmniMapper thread
      omb_.setDebug (true);
      boost::thread omb_thread (&omnimapper::OmniMapperBase::spin, &omb_);
      //boost::thread icp_thread(&omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &icp_plugin_);
      boost::thread csm_thread(&omnimapper::CanonicalScanMatcherPlugin<lscanT>::spin, &csm_plugin_);
    }

    ~OmniMapperPelicanNode(void) {}

    void
    laserScanCallback (const sensor_msgs::LaserScanConstPtr& msg)
    {
      ROS_INFO ("OmniMapperROS got a cloud.");
      //CloudPtr cloud (new Cloud ());
      //pcl::fromROSMsg (*msg, *cloud);
      //icp_plugin_.cloudCallback (cloud);
      //boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr1 (new sensor_msgs::LaserScan);
      boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr1 (new sensor_msgs::LaserScan(*msg));
      csm_plugin_.laserScanCallback (lscanPtr1);
    }


};


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSPelican");
  OmniMapperPelicanNode<lscan> omnimapper;
  ros::spin ();
}


