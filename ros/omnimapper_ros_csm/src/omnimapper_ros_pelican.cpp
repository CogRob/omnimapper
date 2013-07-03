#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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

    // TF Listener
    tf::TransformListener tf_listener_;

    tf::TransformBroadcaster tf_broadcaster_;

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
      //base_frame_name_ = std::string ("/utm30");
      base_frame_name_ = std::string ("/base_link");

      omb_.setSuppressCommitWindow(true);
      tf_plugin_.setTranslationNoise (0.05);
      //tf_plugin_.setRotationNoise (0.1);
      tf_plugin_.setRollNoise (0.0001);
      tf_plugin_.setPitchNoise (0.0001);
      tf_plugin_.setYawNoise (0.05);
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
      publishMapToOdom ();
    }

    void
    publishMapToOdom ()
    {
      gtsam::Pose3 current_pose;
      boost::posix_time::ptime current_time;
      omb_.getLatestPose (current_pose, current_time);
      ros::Time current_time_ros = omnimapper::ptime2rostime (current_time);
      tf::Transform current_pose_ros = Pose3ToTransform (current_pose);

      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
//        tf_listener_.transformPose ("/odom", tf::Stamped<tf::Pose> (tf::btTransform (tf::btQuaternion (current_quat[1], current_quat[2], current_quat[3], current_quat[0]), btVector3 (current_pose.x (), current_pose.y (), current_pose.z ())).inverse (), current_time_ros, "/base_link"), odom_to_map);
        tf_listener_.transformPose ("/odom", tf::Stamped<tf::Pose> (current_pose_ros.inverse (), current_time_ros, "/base_link"), odom_to_map);
      }
      catch (tf::TransformException e)
      {
        ROS_ERROR ("OmniMapperROS: Error with  TF.\n");
        odom_to_map.setIdentity ();
      }
      tf::Transform map_to_odom = odom_to_map.inverse ();//tf::Transform (tf::Quaternion (odom_to_map.getRotation ()), tf::Point (odom_to_map.getOrigin ())
      tf_broadcaster_.sendTransform (tf::StampedTransform (map_to_odom, ros::Time::now (), "/world", "/odom"));
    }

};


int
main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSPelican");
  OmniMapperPelicanNode<lscan> omnimapper;
  ros::spin ();
}


