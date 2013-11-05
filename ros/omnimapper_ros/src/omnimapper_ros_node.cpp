#include <omnimapper/omnimapper_base.h>
#include <omnimapper_ros/tf_pose_plugin.h>
#include <omnimapper/icp_pose_plugin.h>
#include <omnimapper/plane_plugin.h>
#include <omnimapper/object_plugin.h>
#include <omnimapper/no_motion_pose_plugin.h>
#include <omnimapper/tsdf_output_plugin.h>
#include <omnimapper/organized_feature_extraction.h>
#include <omnimapper_ros/omnimapper_visualizer_rviz.h>
#include <omnimapper_ros/OutputMapTSDF.h>
#include <omnimapper_ros/tum_data_error_plugin.h>
#include <omnimapper_ros/error_evaluation_plugin.h>
#include <omnimapper_ros/ros_tf_utils.h>
#include <omnimapper_ros/get_transform_functor_tf.h>
#include <omnimapper_ros/canonical_scan_matcher_plugin.h>
#include <omnimapper_ros/csm_visualizer.h>
#include <omnimapper/time.h>

#include <distortion_model/distortion_model_standalone.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

#include <pcl/io/pcd_grabber.h>
#include <boost/filesystem.hpp>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <google/profiler.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef pcl::PointCloud<pcl::Label> LabelCloud;
typedef typename LabelCloud::Ptr LabelCloudPtr;
typedef typename LabelCloud::ConstPtr LabelCloudConstPtr;

//typedef boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr;
//typedef sensor_msgs::LaserScan lscan;

template<typename PointT>
class OmniMapperROSNode
{
  public:
    // ROS Node Handle
    ros::NodeHandle n_;

    // OmniMapper Instance
    omnimapper::OmniMapperBase omb_;

    // TF Pose Plugin
    omnimapper::TFPosePlugin tf_plugin_;

    // No Motion Pose Plugin
    omnimapper::NoMotionPosePlugin no_motion_plugin_;

    // ICP Plugin
    omnimapper::ICPPoseMeasurementPlugin<PointT> icp_plugin_;

    // Edge ICP Plugin
    omnimapper::ICPPoseMeasurementPlugin<PointT> edge_icp_plugin_;

    // Plane Plugin
    omnimapper::PlaneMeasurementPlugin<PointT> plane_plugin_;

    // Object Plugin
    omnimapper::ObjectPlugin<PointT> object_plugin_;

    //CSM Plugin
    omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan> csm_plugin_;

    // Visualization
    omnimapper::OmniMapperVisualizerRViz<PointT> vis_plugin_;

    // CSM Visualization
    omnimapper::CSMVisualizerRViz<sensor_msgs::LaserScan> csm_vis_plugin_;

    // TSDF Plugin
    omnimapper::TSDFOutputPlugin<PointT> tsdf_plugin_;

    // Benchmark Data Analysis
    omnimapper::TUMDataErrorPlugin bag_error_plugin_;

    omnimapper::ErrorEvaluationPlugin eval_plugin_;

    // Distortion Model
    DistortionModelStandalone distortion_model_;

    // Fake Grabber (TODO: Fix this)
    std::vector<std::string> empty_files_;
    pcl::PCDGrabber<PointT> fake_grabber_;
    // Organized Feature Extraction
    omnimapper::OrganizedFeatureExtraction<PointT> organized_feature_extraction_;

    // TF Listener  (for initialization)
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // Subscribers
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber laserScan_sub_;

    ros::ServiceServer generate_tsdf_srv_;

    // Mapper config
    bool use_planes_;
    bool use_objects_;
    bool use_csm_;
    bool use_icp_;
    bool use_occ_edge_icp_;
    bool use_tf_;
    bool use_no_motion_;

    // ROS Params
    std::string odom_frame_name_;
    std::string base_frame_name_;
    std::string rgbd_frame_name_;
    std::string cloud_topic_name_;

    bool init_pose_from_tf_;
    bool use_init_pose_;
    double init_x_, init_y_, init_z_;
    double init_qx_, init_qy_, init_qz_, init_qw_;

    // ICP Params
    double icp_leaf_size_;
    double icp_max_correspondence_distance_;
    double icp_trans_noise_;
    double icp_rot_noise_;
    bool icp_add_identity_on_fail_;
    bool icp_add_loop_closures_;

    // Occluding Edge ICP Params
    double occ_edge_trans_noise_;
    double occ_edge_rot_noise_;
    double occ_edge_score_thresh_;
    double occ_edge_max_correspondence_dist_;
    bool occ_edge_add_identity_on_fail_;

    // Plane Plugin Params
    double plane_range_threshold_;
    double plane_angular_threshold_;
    double plane_range_noise_;
    double plane_angular_noise_;

    // Object Plugin Params
    std::string object_database_location_;
    bool object_loop_closures_;
    bool object_landmarks_;
    bool save_object_models_;
    double object_min_height_;

    // Labelled Cloud Plugin Params
    bool use_label_cloud_;

    // TF Plugin Params
    double tf_trans_noise_;
    double tf_rot_noise_;
    double tf_roll_noise_, tf_pitch_noise_, tf_yaw_noise_;

    // Visualization params
    bool use_rviz_plugin_;
    bool draw_pose_array_;
    bool draw_pose_graph_;
    bool draw_label_cloud_;
    bool draw_clusters_;

    // TSDF plugin params
    bool use_tsdf_plugin_;

    // Error plugin params
    bool use_error_plugin_;

    // Other Flags
    bool add_pose_per_cloud_;
    bool broadcast_map_to_odom_;
    bool use_distortion_model_;
    bool use_rgbd_sensor_base_tf_functor_;
    std::string distortion_model_path_;
    bool debug_;

    // Evaluation Mode
    bool evaluation_mode_;
    std::string evaluation_pcd_path_;
    std::vector<std::string> evaluation_pcd_files_;
    int evaluation_file_idx_;
    // Note: we require the associated_txt path because the PCD format does not include timestamps.
    // We instead load the timestamps from this file.
    std::string evaluation_associated_txt_path_;
    std::string evaluation_ground_truth_txt_path_;
    std::string evaluation_output_trajectory_txt_path_;
    bool evaluation_mode_write_trajectory_;
    bool evaluation_mode_write_tsdf_;
    ros::Timer eval_timer_;
    bool evaluation_mode_paused_;
    bool use_organized_feature_extraction_;
    bool evaluation_show_frames_;

    OmniMapperROSNode ()
      : n_ ("~"),
        omb_ (),
        tf_plugin_ (&omb_),
        no_motion_plugin_ (&omb_),
        icp_plugin_ (&omb_),
        edge_icp_plugin_ (&omb_),
        plane_plugin_ (&omb_),
        object_plugin_ (&omb_),
        csm_plugin_ (&omb_),
        vis_plugin_ (&omb_),
        csm_vis_plugin_ (&omb_),
        tsdf_plugin_ (&omb_),
        bag_error_plugin_ (&omb_),
        eval_plugin_ (&omb_),
        fake_grabber_ (empty_files_, 1.0, false),
        organized_feature_extraction_ (fake_grabber_),
        tf_listener_ (ros::Duration (500.0))
    {
      // Load some params
      n_.param ("use_planes", use_planes_, true);
      n_.param ("use_objects", use_objects_, true);
      n_.param ("use_csm", use_csm_, true);
      n_.param ("use_icp", use_icp_, true);
      n_.param ("use_occ_edge_icp", use_occ_edge_icp_, true);
      n_.param ("use_tf", use_tf_, true);
      n_.param ("use_tsdf_plugin", use_tsdf_plugin_, true);
      n_.param ("use_error_plugin", use_error_plugin_, false);
      n_.param ("use_no_motion", use_no_motion_, false);
      n_.param ("odom_frame_name", odom_frame_name_, std::string ("/odom"));
      n_.param ("base_frame_name", base_frame_name_,
          std::string ("/camera_depth_optical_frame"));
      n_.param ("cloud_topic_name", cloud_topic_name_,
          std::string ("/throttled_points"));
      n_.param ("rgbd_frame_name", rgbd_frame_name_,
          std::string ("/camera_rgb_optical_frame"));
      n_.param ("icp_leaf_size", icp_leaf_size_, 0.05);
      n_.param ("icp_max_correspondence_distance",
          icp_max_correspondence_distance_, 0.5);
      n_.param ("icp_trans_noise", icp_trans_noise_, 0.1);
      n_.param ("icp_rot_noise", icp_rot_noise_, 0.1);
      n_.param ("icp_add_identity_on_fail", icp_add_identity_on_fail_, false);
      n_.param ("icp_add_loop_closures", icp_add_loop_closures_, true);
      n_.param ("occ_edge_trans_noise", occ_edge_trans_noise_, 0.1);
      n_.param ("occ_edge_rot_noise", occ_edge_rot_noise_, 0.1);
      n_.param ("occ_edge_score_thresh", occ_edge_score_thresh_, 0.1);
      n_.param ("occ_edge_max_correspondence_dist",
          occ_edge_max_correspondence_dist_, 0.1);
      n_.param ("occ_edge_add_identity_on_fail", occ_edge_add_identity_on_fail_,
          false);
      n_.param ("plane_range_threshold", plane_range_threshold_, 0.6);
      n_.param ("plane_angular_threshold", plane_angular_threshold_,
          pcl::deg2rad (10.0));
      n_.param ("plane_range_noise", plane_range_noise_, 0.2);
      n_.param ("plane_angular_noise", plane_angular_noise_, 0.26);
      n_.param ("tf_trans_noise", tf_trans_noise_, 0.05);
      n_.param ("tf_rot_noise", tf_rot_noise_, pcl::deg2rad (10.0));
      n_.param ("tf_roll_noise", tf_roll_noise_, tf_rot_noise_);
      n_.param ("tf_pitch_noise", tf_pitch_noise_, tf_rot_noise_);
      n_.param ("tf_yaw_noise", tf_yaw_noise_, tf_rot_noise_);
      n_.param ("use_init_pose", use_init_pose_, false);
      n_.param ("init_pose_from_tf", init_pose_from_tf_, false);
      n_.param ("init_x", init_x_, 0.0);
      n_.param ("init_y", init_y_, 0.0);
      n_.param ("init_z", init_z_, 0.0);
      n_.param ("init_qx", init_qx_, 0.0);
      n_.param ("init_qy", init_qy_, 0.0);
      n_.param ("init_qz", init_qz_, 0.0);
      n_.param ("init_qw", init_qw_, 1.0);
      n_.param ("use_rviz_plugin", use_rviz_plugin_, true);
      n_.param ("draw_pose_array", draw_pose_array_, true);
      n_.param ("draw_pose_graph", draw_pose_graph_, true);
      n_.param ("draw_label_cloud", draw_label_cloud_, true);
      n_.param ("use_label_cloud", use_label_cloud_, true);
      n_.param ("add_pose_per_cloud", add_pose_per_cloud_, true);
      n_.param ("broadcast_map_to_odom", broadcast_map_to_odom_, false);
      n_.param ("use_distortion_model", use_distortion_model_, false);
      n_.param ("use_rgbd_sensor_base_tf_functor",
          use_rgbd_sensor_base_tf_functor_, true);
      n_.param ("distortion_model_path", distortion_model_path_,
          std::string (
              "/home/atrevor/github/atrevor_sandbox/sdmiller_calibration/new_distortion_model"));
      n_.param ("evaluation_mode", evaluation_mode_, false);
      n_.param ("evaluation_pcd_path", evaluation_pcd_path_, std::string (""));
      n_.param ("evaluation_associated_txt_path",
          evaluation_associated_txt_path_, std::string (""));
      n_.param ("evaluation_ground_truth_txt_path",
          evaluation_ground_truth_txt_path_, std::string (""));
      n_.param ("evaluation_output_trajectory_txt_path",
          evaluation_output_trajectory_txt_path_, std::string (""));
      n_.param ("evaluation_mode_write_trajectory",
          evaluation_mode_write_trajectory_, true);
      n_.param ("evaluation_mode_write_tsdf", evaluation_mode_write_tsdf_,
          false);
      n_.param ("evaluation_mode_paused", evaluation_mode_paused_, false);
      n_.param ("evaluation_show_frames", evaluation_show_frames_, true);
      n_.param ("object_database_location", object_database_location_,
          std::string ("/home/siddharth/kinect/"));
      n_.param ("object_loop_closures", object_loop_closures_, true);
      n_.param ("object_landmarks", object_landmarks_, true);
      n_.param ("save_object_models", save_object_models_, true);
      n_.param ("object_min_height", object_min_height_, 0.3);
      n_.param ("use_organized_feature_extraction", use_organized_feature_extraction_, true);
      n_.param ("debug", debug_, false);

      // Optionally specify an alternate initial pose
      if (use_init_pose_)
      {
        gtsam::Pose3 init_pose (
            gtsam::Rot3::quaternion (init_qw_, init_qx_, init_qy_, init_qz_),
            gtsam::Point3 (init_x_, init_y_, init_z_));
        omb_.setInitialPose (init_pose);
      }

      // Optionally get initial pose from TF
      if (init_pose_from_tf_)
      {
        bool got_tf = false;
        tf::StampedTransform init_transform;

        while (!got_tf)
        {
          got_tf = true;
          try
          {
            ROS_INFO("Waiting for initial pose from %s to %s",
                odom_frame_name_.c_str (), base_frame_name_.c_str ());
            tf_listener_.waitForTransform (odom_frame_name_, base_frame_name_,
                ros::Time::now (), ros::Duration (1.0));
            tf_listener_.lookupTransform (odom_frame_name_, base_frame_name_,
                ros::Time::now (), init_transform);
          }
          catch (tf::TransformException ex)
          {
            ROS_INFO("Transform not yet available!");
            got_tf = false;
          }
        }

        gtsam::Pose3 init_pose = omnimapper::tf2pose3 (init_transform);
        gtsam::Pose3 init_pose_inv = init_pose.inverse ();
        omb_.setInitialPose (init_pose_inv);
      }

      // Use ROS Time instead of system clock
      omnimapper::GetTimeFunctorPtr time_functor_ptr (
          new omnimapper::GetROSTimeFunctor ());
      omb_.setTimeFunctor (time_functor_ptr);
      omb_.setSuppressCommitWindow (false);

      // Optionally use distortion model
      if (use_distortion_model_)
      {
        distortion_model_.load (distortion_model_path_);
      }

      // Add the TF Pose Plugin
      tf_plugin_.setOdomFrameName (odom_frame_name_);
      tf_plugin_.setBaseFrameName (base_frame_name_);
      tf_plugin_.setRotationNoise (tf_trans_noise_);    //0.1
      tf_plugin_.setRollNoise (tf_roll_noise_);
      tf_plugin_.setPitchNoise (tf_pitch_noise_);
      tf_plugin_.setYawNoise (tf_yaw_noise_);
      tf_plugin_.setTranslationNoise (tf_rot_noise_);    //0.1
      if (use_tf_)
      {
        boost::shared_ptr<omnimapper::PosePlugin> tf_plugin_ptr (&tf_plugin_);
        omb_.addPosePlugin (tf_plugin_ptr);
      }

      // Add the No Motion Plugin (null motion model)
      if (use_no_motion_)
      {
        boost::shared_ptr<omnimapper::PosePlugin> no_motion_ptr (
            &no_motion_plugin_);
        omb_.addPosePlugin (no_motion_ptr);
      }

      // Set up a sensor_to_base functor, for plugins to use
      //omnimapper::GetTransformFunctorTF rgbd_to_base (std::string ("/camera_rgb_optical_frame"), std::string ("/base_link"));
      //omnimapper::GetTransformFunctorPtr rgbd_to_base_ptr (&rgbd_to_base);
      omnimapper::GetTransformFunctorPtr rgbd_to_base_ptr (
          new omnimapper::GetTransformFunctorTF (rgbd_frame_name_,
              base_frame_name_));
      // Optionally disable this, if we don't have TF available
      if (!use_rgbd_sensor_base_tf_functor_)
      {
        rgbd_to_base_ptr = omnimapper::GetTransformFunctorPtr (
            new omnimapper::GetTransformFunctorIdentity ());
      }

      // Set up an ICP Plugin
      icp_plugin_.setUseGICP (true);
      icp_plugin_.setOverwriteTimestamps (false);
      icp_plugin_.setAddIdentityOnFailure (icp_add_identity_on_fail_);
      icp_plugin_.setShouldDownsample (true);
      icp_plugin_.setLeafSize (icp_leaf_size_);      //0.02
      icp_plugin_.setMaxCorrespondenceDistance (
          icp_max_correspondence_distance_);
      icp_plugin_.setScoreThreshold (0.8);
      icp_plugin_.setTransNoise (icp_trans_noise_);//10.1
      icp_plugin_.setRotNoise (icp_rot_noise_);//10.1
      icp_plugin_.setAddLoopClosures (icp_add_loop_closures_);
      icp_plugin_.setTransNoise (icp_trans_noise_);      //10.1
      icp_plugin_.setRotNoise (icp_rot_noise_);      //10.1
      icp_plugin_.setAddLoopClosures (true);
      icp_plugin_.setLoopClosureDistanceThreshold (1.0);
      icp_plugin_.setSaveFullResClouds (true);
      icp_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);

      // Set up edge ICP plugin
      edge_icp_plugin_.setUseGICP (false);
      edge_icp_plugin_.setOverwriteTimestamps (false);
      edge_icp_plugin_.setAddIdentityOnFailure (occ_edge_add_identity_on_fail_);
      edge_icp_plugin_.setShouldDownsample (false);
      edge_icp_plugin_.setMaxCorrespondenceDistance (
          occ_edge_max_correspondence_dist_);
      edge_icp_plugin_.setScoreThreshold (occ_edge_score_thresh_);
      edge_icp_plugin_.setTransNoise (occ_edge_trans_noise_);      //10.1
      edge_icp_plugin_.setRotNoise (occ_edge_rot_noise_);      //10.1
      edge_icp_plugin_.setAddLoopClosures (false);
      edge_icp_plugin_.setLoopClosureDistanceThreshold (0.15);
      edge_icp_plugin_.setSaveFullResClouds (false);
      edge_icp_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);

      // Set up the Plane Plugin
      plane_plugin_.setOverwriteTimestamps (false);
      plane_plugin_.setDisableDataAssociation (false);
      plane_plugin_.setRangeThreshold (plane_range_threshold_);
      plane_plugin_.setAngularThreshold (plane_angular_threshold_);
      //plane_plugin_.setAngularNoise (1.1);
      plane_plugin_.setAngularNoise (plane_angular_noise_);      //0.26
      //plane_plugin_.setRangeNoise (2.2);
      plane_plugin_.setRangeNoise (plane_range_noise_);      //0.2
      plane_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);

      // Set up the object Plugin
      if (use_objects_)
      {
        object_plugin_.setSensorToBaseFunctor (rgbd_to_base_ptr);
        object_plugin_.setAndLoadObjectDatabaseLocation (
            object_database_location_);
        object_plugin_.useObjectLoopClosures (object_loop_closures_);
        object_plugin_.useObjectLandmarks (object_landmarks_);
        object_plugin_.saveObjectModels (save_object_models_);
        object_plugin_.setMinimumClusterHeight (object_min_height_);  //min height of the object above floor
      }

      // Set up the feature extraction
      if (use_occ_edge_icp_)
      {
        boost::function<void (const CloudConstPtr&)> edge_icp_cloud_cb =
            boost::bind (
                &omnimapper::ICPPoseMeasurementPlugin<PointT>::cloudCallback,
                &edge_icp_plugin_, _1);
        organized_feature_extraction_.setOccludingEdgeCallback (
            edge_icp_cloud_cb);
      }

      if (use_planes_)
      {
        boost::function<
            void (
                std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
                omnimapper::Time)> plane_cb = boost::bind (
            &omnimapper::PlaneMeasurementPlugin<PointT>::planarRegionCallback,
            &plane_plugin_, _1, _2);
        organized_feature_extraction_.setPlanarRegionStampedCallback (plane_cb);
        boost::function<
            void (
                std::vector<pcl::PlanarRegion<PointT>,
                    Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >,
                omnimapper::Time)> plane_vis_cb = boost::bind (
            &omnimapper::OmniMapperVisualizerRViz<PointT>::planarRegionCallback,
            &vis_plugin_, _1, _2);
        organized_feature_extraction_.setPlanarRegionStampedCallback (
            plane_vis_cb);
      }

      // Optionally draw label cloud
      if (draw_label_cloud_)
      {
        boost::function<void (const CloudConstPtr&, const LabelCloudConstPtr&)> label_vis_callback =
            boost::bind (
                &omnimapper::OmniMapperVisualizerRViz<PointT>::labelCloudCallback,
                &vis_plugin_, _1, _2);
        organized_feature_extraction_.setClusterLabelsCallback (
            label_vis_callback);
      }

      // Optionally draw clusters
      if (draw_clusters_)
      {
        boost::function<
            void (std::vector<CloudPtr>, omnimapper::Time t,
                boost::optional<std::vector<pcl::PointIndices> >)> cluster_vis_callback =
            boost::bind (
                &omnimapper::OmniMapperVisualizerRViz<PointT>::clusterCloudCallback,
                &vis_plugin_, _1, _2, _3);
        organized_feature_extraction_.setClusterCloudCallback (
            cluster_vis_callback);

        //gtsam::Symbol, boost::optional<gtsam::Pose3>, std::vector<CloudPtr>, omnimapper::Time t

      }

      // Optionally use labels
      if (use_objects_)
      {
        typename boost::function<
            void (std::vector<CloudPtr>, omnimapper::Time t,
                boost::optional<std::vector<pcl::PointIndices> >)> object_cluster_callback =
            boost::bind (
                &omnimapper::ObjectPlugin<PointT>::clusterCloudCallback,
                &object_plugin_, _1, _2, _3);
        organized_feature_extraction_.setClusterCloudCallback (
            object_cluster_callback);

        boost::function<
            void (
                std::map<gtsam::Symbol, omnimapper::Object<PointT> > object_map,
                gtsam::Point3, gtsam::Point3)> object_vis_callback =
            boost::bind (
                &omnimapper::OmniMapperVisualizerRViz<PointT>::objectCallback,
                &vis_plugin_, _1, _2, _3);
        object_plugin_.setObjectCallback (object_vis_callback);
        // 	object_plugin_.test();

      }

      // Canonical Scan Matcher
      if (use_csm_)
      {
        // Subscribe to laser scan
        laserScan_sub_ = n_.subscribe ("/scan", 1,
            &OmniMapperROSNode::laserScanCallback, this);

        boost::shared_ptr<
            omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan> > csm_ptr (
            &csm_plugin_);
        csm_vis_plugin_.setCSMPlugin (csm_ptr);

        // Install the visualizer
        boost::shared_ptr<omnimapper::OutputPlugin> csm_vis_ptr (
            &csm_vis_plugin_);
        omb_.addOutputPlugin (csm_vis_ptr);

        boost::thread csm_thread (
            &omnimapper::CanonicalScanMatcherPlugin<sensor_msgs::LaserScan>::spin,
            &csm_plugin_);
      }

      // Set the ICP Plugin on the visualizer
      boost::shared_ptr<omnimapper::ICPPoseMeasurementPlugin<PointT> > icp_ptr (
          &icp_plugin_);
      vis_plugin_.setICPPlugin (icp_ptr);

      // Set up the Object Plugin with the visualizer
      boost::shared_ptr<omnimapper::ObjectPlugin<PointT> > obj_ptr (
          &object_plugin_);
      vis_plugin_.setObjectPlugin (obj_ptr);

      // Subscribe to Point Clouds
      pointcloud_sub_ = n_.subscribe (cloud_topic_name_, 1,
          &OmniMapperROSNode::cloudCallback, this);  //("/camera/depth/points", 1, &OmniMapperHandheldNode::cloudCallback, this);

      // Install the visualizer
      if (use_rviz_plugin_)
      {
        vis_plugin_.setDrawPoseArray (draw_pose_array_);
        vis_plugin_.setDrawPoseGraph (draw_pose_graph_);
        boost::shared_ptr<omnimapper::OutputPlugin> vis_ptr (&vis_plugin_);
        omb_.addOutputPlugin (vis_ptr);
      }

      // Set up the TSDF Plugin
      if (use_tsdf_plugin_)
      {
        tsdf_plugin_.setICPPlugin (icp_ptr);
        boost::shared_ptr<omnimapper::OutputPlugin> tsdf_ptr (&tsdf_plugin_);
        omb_.addOutputPlugin (tsdf_ptr);
      }

      // Set up the error Plugin
      if (use_error_plugin_)
      {
        boost::shared_ptr<omnimapper::OutputPlugin> error_plugin_ptr (
            &bag_error_plugin_);
        omb_.addOutputPlugin (error_plugin_ptr);
      }

      generate_tsdf_srv_ = n_.advertiseService ("generate_map_tsdf",
          &OmniMapperROSNode::generateMapTSDFCallback, this);

      //OmniMapper thread
      omb_.setDebug (false);
      boost::thread omb_thread (&omnimapper::OmniMapperBase::spin, &omb_);
      if (use_icp_)
        boost::thread icp_thread (
            &omnimapper::ICPPoseMeasurementPlugin<PointT>::spin, &icp_plugin_);
      if (use_occ_edge_icp_)
        boost::thread edge_icp_thread (
            &omnimapper::ICPPoseMeasurementPlugin<PointT>::spin,
            &edge_icp_plugin_);
      // if (use_planes_)
      //   boost::thread plane_thread (&omnimapper::PlaneMeasurementPlugin<PointT>::spin, &plane_plugin_);

      // If evaluation mode, start a timer to check on things, and load the files
      if (evaluation_mode_)
      {

        // Set up the error evaluation plugin
        eval_plugin_.loadAssociatedFile (evaluation_associated_txt_path_);
        eval_plugin_.loadGroundTruthFile (evaluation_ground_truth_txt_path_);

        boost::shared_ptr<omnimapper::OutputPlugin> eval_plugin_ptr (
            &eval_plugin_);
        omb_.addOutputPlugin (eval_plugin_ptr);  // Calls the update function

        // Set up Visualization and Interactive Markers
        printf ("Getting interactive ptrs\n");
        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> ims_ptr =
            vis_plugin_.getInteractiveMarkerServerPtr ();
        boost::shared_ptr<interactive_markers::MenuHandler> mh_ptr =
            vis_plugin_.getMenuHandlerPtr ();
        printf ("setting ptrs\n");
        eval_plugin_.setInteractiveMarkerServerPtr (ims_ptr);
        eval_plugin_.setMenuHandlerPtr (mh_ptr);
        printf ("done with that\n");

        // Initialize pose to start in the same coord system as the ground truth
        gtsam::Pose3 gt_init = eval_plugin_.getInitialPose ();
        omb_.setInitialPose (gt_init);

        std::cout << "Loading PCDs" << std::endl;
        evaluation_file_idx_ = 0;
        boost::filesystem::directory_iterator end_itr;
        for (boost::filesystem::directory_iterator itr (evaluation_pcd_path_);
            itr != end_itr; ++itr)
        {
          if (itr->path ().extension () == ".pcd")
            evaluation_pcd_files_.push_back (itr->path ().string ());
        }
        sort (evaluation_pcd_files_.begin (), evaluation_pcd_files_.end ());
        ROS_INFO("OmniMapper: Loaded %d files for evaluation\n",
            evaluation_pcd_files_.size ());

        eval_timer_ = n_.createTimer (ros::Duration (0.01), &OmniMapperROSNode::evalTimerCallback, this);
/*
        while (ros::ok ())
        {
          std::cout << ros::Time::now () << std::endl;
          evalTimerCallback ();
          ros::Duration (0.001).sleep ();
        }*/
      }

    }

    void cloudCallback (const sensor_msgs::PointCloud2ConstPtr& msg)
    {
      if (debug_)
        ROS_INFO("OmniMapperROS got a cloud.");
      double start_cb = pcl::getTime ();
      double start_copy = pcl::getTime ();
      CloudPtr cloud (new Cloud ());
      pcl::fromROSMsg<PointT> (*msg, *cloud);
      double end_copy = pcl::getTime ();
      omnimapper::Time cloud_stamp = omnimapper::stamp2ptime (
          cloud->header.stamp);
      if (debug_)
      {
        std::cout << "OmniMapperRos: Got cloud from: " << cloud_stamp
                  << std::endl;
      }
      //cloud->header.stamp = msg->header.stamp.toBoost ();
      if (debug_)
      {
        std::cout << "cloudCallback: conversion took "
                  << double (end_copy - start_copy) << std::endl;
      }
      
      //pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>());
      //pcl::fromROSMsg (*msg, *xyz_cloud);
      //CloudPtr cloud (new Cloud ());
      // pcl::copyPointCloud (*xyz_cloud, *cloud);

      if (use_distortion_model_)
      {
        double start_undistort = pcl::getTime ();
        distortion_model_.undistort (*cloud);
        double end_undistort = pcl::getTime ();
        if (debug_)
        {
          std::cout << "cloudCallback: undistortion took "
                    << double (end_undistort - start_undistort) << std::endl;
        }
        
      }

      if (use_icp_)
      {
        if (debug_)
        {
          std::cout << "Calling ICP Plugin with stamp: "
                    << omnimapper::stamp2ptime (cloud->header.stamp) << std::endl;
        }
        
        icp_plugin_.cloudCallback (cloud);
      }
      
      if (use_organized_feature_extraction_)
      {
        double start_ofe = pcl::getTime ();
        organized_feature_extraction_.cloudCallback (cloud);
        double end_ofe = pcl::getTime ();
        if (debug_)
          std::cout << "cloudCallback: ofe_cb took " << double(end_ofe - start_ofe) << std::endl;
      }

      if (add_pose_per_cloud_)
      {
        double start_getpose = pcl::getTime ();
        gtsam::Symbol sym;
        boost::posix_time::ptime header_time = omnimapper::stamp2ptime (
            cloud->header.stamp);      //msg->header.stamp.toBoost ();
        if (debug_)
          std::cout << "header time: " << header_time << std::endl;
        omb_.getPoseSymbolAtTime (header_time, sym);
        double end_getpose = pcl::getTime ();
        if (debug_)
          std::cout << "cloudCallback: get_pose took " << double (end_getpose - start_getpose) << std::endl;
      }
      if (broadcast_map_to_odom_)
      {
        double start_pub = pcl::getTime ();
        publishMapToOdom ();
        double end_pub = pcl::getTime ();
        if (debug_)
          std::cout << "cloudCallback: pub took " << double (end_pub - start_pub) << std::endl;
      }
      double end_cb = pcl::getTime ();
      if (debug_)
        std::cout << "cloudCallback: cb took: " << double (end_cb - start_cb) << std::endl;
    }

    void laserScanCallback (const sensor_msgs::LaserScanConstPtr& msg)
    {
      ROS_INFO("OmniMapperROS got a cloud.");
      //CloudPtr cloud (new Cloud ());
      //pcl::fromROSMsg (*msg, *cloud);
      //icp_plugin_.cloudCallback (cloud);
      //boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr1 (new sensor_msgs::LaserScan);

      gtsam::Symbol sym;
      boost::posix_time::ptime header_time = msg->header.stamp.toBoost ();
      omb_.getPoseSymbolAtTime (header_time, sym);

      boost::shared_ptr<sensor_msgs::LaserScan> lscanPtr1 (
          new sensor_msgs::LaserScan (*msg));
      csm_plugin_.laserScanCallback (lscanPtr1);
      publishMapToOdom ();
    }

    bool generateMapTSDFCallback (omnimapper_ros::OutputMapTSDF::Request& req,
        omnimapper_ros::OutputMapTSDF::Response &res)
    {
      printf ("TSDF Service call, calling tsdf plugin\n");
      tsdf_plugin_.generateTSDF (req.grid_size, req.resolution);
      return (true);
    }

    void evalTimerCallback (const ros::TimerEvent& e)
    {
      double cb_start = pcl::getTime ();

      if (debug_)
        ROS_INFO("In timer callback at %lf!\n", cb_start);

      // Check if everything is done processing
      bool ready = true;
      if (use_icp_ && !icp_plugin_.ready ())
        ready = false;

      if (use_occ_edge_icp_ && !edge_icp_plugin_.ready ())
        ready = false;

      if (evaluation_mode_paused_)
        ready = false;

      if (ready && (evaluation_file_idx_ < evaluation_pcd_files_.size ()))
      {

        // Load next file
        CloudPtr cloud (new Cloud ());
        double load_start = pcl::getTime ();
        pcl::io::loadPCDFile (evaluation_pcd_files_[evaluation_file_idx_],
            *cloud);
        double load_end = pcl::getTime ();
        
        if (debug_)
        {
          std::cout << "Loading took: " << double (load_end - load_start) << std::endl;
          
          ROS_INFO("Processing cloud %d with %d points\n", evaluation_file_idx_,
                   cloud->points.size ());
        }

        // Convert it
        sensor_msgs::PointCloud2Ptr cloud_msg (new sensor_msgs::PointCloud2 ());
        // We have to get the timestamp from associated.txt (parsed by eval plugin) since PCD doesn't include timestamps...
        cloud->header.stamp = eval_plugin_.getStampFromIndex (
            evaluation_file_idx_);
        pcl::toROSMsg (*cloud, *cloud_msg);

        evaluation_file_idx_++;

        sensor_msgs::PointCloud2ConstPtr cloud_msg_ptr (cloud_msg);
        cloudCallback (cloud_msg_ptr);

        // TODO: Visualization adds a lag  each time it is called, delays if other plugins are ready or not
        if (evaluation_show_frames_)
        {
          eval_plugin_.visualizeEachFrame (cloud);  // visualize each frame
          eval_plugin_.visualizeStats ();
        }

        ready = false;
      }
      else
      {
        if (debug_)
          ROS_INFO("Plugins not yet ready.\n");
      }
      
      // Write the trajectory if we're finished
      if (ready && (evaluation_file_idx_ == evaluation_pcd_files_.size ()))
      {
        ROS_INFO("Completed evaluation, writing output file.");
        gtsam::Values solution = omb_.getSolution ();

        if (evaluation_mode_write_trajectory_)
        {
          eval_plugin_.writeMapperTrajectoryFile (
              evaluation_output_trajectory_txt_path_, solution);
          ROS_INFO("Evaluation log written as: %s",
              evaluation_output_trajectory_txt_path_.c_str ());
        }

        if (evaluation_mode_write_tsdf_)
        {
          tsdf_plugin_.generateTSDF (10.0, 1024);
        }

        exit (0);
      }

      double cb_end = pcl::getTime ();
      if (debug_)
        std::cout << "Timer callback took: " << double (cb_end - cb_start) << std::endl;
    }

    void publishMapToOdom ()
    {
      gtsam::Pose3 current_pose;
      boost::posix_time::ptime current_time;
      omb_.getLatestPose (current_pose, current_time);
      ros::Time current_time_ros = omnimapper::ptime2rostime (current_time);
      tf::Transform current_pose_ros = omnimapper::pose3totf (current_pose);

      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
//        tf_listener_.transformPose ("/odom", tf::Stamped<tf::Pose> (tf::btTransform (tf::btQuaternion (current_quat[1], current_quat[2], current_quat[3], current_quat[0]), btVector3 (current_pose.x (), current_pose.y (), current_pose.z ())).inverse (), current_time_ros, "/base_link"), odom_to_map);
        tf_listener_.waitForTransform (odom_frame_name_, base_frame_name_,
            current_time_ros, ros::Duration (0.02));
        tf_listener_.transformPose (odom_frame_name_,
            tf::Stamped<tf::Pose> (current_pose_ros.inverse (),
                current_time_ros, base_frame_name_), odom_to_map);
      }
      catch (tf::TransformException e)
      {
        ROS_ERROR(
            "OmniMapperROS: Error: could not immediately get odom to base transform\n");
        odom_to_map.setIdentity ();
        return;
      }
      tf::Transform map_to_odom = odom_to_map.inverse ();  //tf::Transform (tf::Quaternion (odom_to_map.getRotation ()), tf::Point (odom_to_map.getOrigin ())
      tf_broadcaster_.sendTransform (
          tf::StampedTransform (map_to_odom, ros::Time::now (), "/world",
              odom_frame_name_));
    }

    // void
    // labelCloudCallback (const CloudConstPtr& cloud, const LabelCloudConstPtr& labels)
    // {

    // }

};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "OmniMapperROSNode");
  //ProfilerStart ("omnimapper_ros_node.prof");
  OmniMapperROSNode<PointT> omnimapper;
  ros::spin ();
  //ProfilerStop ();
}
