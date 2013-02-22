#include <omnimapper/omnimapper_visualizer_pcl.h>
#include <pcl/common/transforms.h>

template <typename PointT>
omnimapper::OmniMapperVisualizerPCL<PointT>::OmniMapperVisualizerPCL (omnimapper::OmniMapperBase* mapper) 
  : mapper_ (mapper),
    viewer_ ("OmniMapperVisualizerPCL"),
    pose_cloud_ (new pcl::PointCloud<pcl::PointXYZ> ()),
    new_slam_data_ (false),
    draw_icp_clouds_ (false)
{
  // Set up the viewer
  viewer_.setBackgroundColor (0, 0, 0);
  viewer_.addCoordinateSystem (1.0);
  viewer_.initCameraParameters ();
  viewer_.registerKeyboardCallback (&omnimapper::OmniMapperVisualizerPCL<PointT>::keyboardCallback, *this, 0);
  debug_ = true;
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerPCL<PointT>::update (boost::shared_ptr<gtsam::Values>& vis_values)
{
  // Pull poses from the mapper
  //gtsam::Values current_solution = mapper_->getSolution ();
  gtsam::Values current_solution = *vis_values;

  // Draw the poses
  pcl::PointCloud<pcl::PointXYZ>::Ptr poses_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
  {
    gtsam::Symbol key_symbol (key_value.key);
    gtsam::Pose3 pose = key_value.value;
    gtsam::Rot3 rot = pose.rotation ();
    
    pcl::PointXYZ pose_pt (pose.x (), pose.y (), pose.z ());
    poses_cloud->push_back (pose_pt);
    
    // Draw the clouds
    if (draw_icp_clouds_)
    {
      printf ("About to request frame cloud\n");
      CloudConstPtr frame_cloud = icp_plugin_->getCloudPtr (key_symbol);
      printf ("Frame cloud has %d\n", frame_cloud->points.size ());
      char frame_name[1024];
      CloudPtr map_cloud (new Cloud ());
      Eigen::Matrix4f map_tform = pose.matrix ().cast<float>();
      pose.print ("SAM Pose: ");
      std::cout << "Map Tform: " << map_tform << std::endl;
      pcl::transformPointCloud (*frame_cloud, *map_cloud, map_tform);
      sprintf (frame_name, "x_%d", key_symbol.index ());
      printf ("name: x_%d\n",key_symbol.index ());
      if (!viewer_.updatePointCloud (map_cloud, frame_name))
        viewer_.addPointCloud (map_cloud, frame_name);  
      viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.05, frame_name);
    }
  }
  
  {
    boost::mutex::scoped_lock (vis_mutex_);
    
    if (debug_)
      printf ("Visualizer updating with %d poses\n", poses_cloud->points.size ());

    if (poses_cloud->points.size () > 0)
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (poses_cloud, 0, 255, 0);
      if (!viewer_.updatePointCloud (poses_cloud, color, "poses_cloud"))
        viewer_.addPointCloud (poses_cloud, color, "poses_cloud");
      viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "poses_cloud");
    }

    //pose_cloud_ = poses_cloud;
    new_slam_data_ = true;
    draw_icp_clouds_ = false;
  }
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerPCL<PointT>::keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*)
{
  if (event.keyUp ())
  {
    switch (event.getKeyCode ())
    {  
      case 'c':
      case 'C':
        //Draw Clouds
        draw_icp_clouds_ = true;
        break;
      case 'p':
        icp_plugin_->pause (true);
        break;
      case ' ':
        icp_plugin_->pause (false);
        break;
    }
    
  }
  
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerPCL<PointT>::spinOnce ()
{
  viewer_.spinOnce (); 
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerPCL<PointT>::spin ()
{
  boost::thread (&omnimapper::OmniMapperVisualizerPCL<PointT>::spinThread, this);
}

template <typename PointT> void
omnimapper::OmniMapperVisualizerPCL<PointT>::spinThread ()
{
  while (!viewer_.wasStopped ())
  {
    viewer_.spinOnce (100);

    if (vis_mutex_.try_lock ())
    {
      if (new_slam_data_)
      {
        printf ("Updating vis\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color (pose_cloud_, 0, 255, 0);
        if (!viewer_.updatePointCloud (pose_cloud_, color, "poses_cloud"))
          viewer_.addPointCloud (pose_cloud_, color, "poses_cloud");
        viewer_.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "poses_cloud");   
        //viewer_.spinOnce ();
        new_slam_data_ = false;
      }
      vis_mutex_.unlock ();
    } else 
    {
      if (debug_)
        printf ("Vis spin thread: Couldn't get lock!\n");
    }
    
  }
  
}

// TODO: Instantiation macros.
template class omnimapper::OmniMapperVisualizerPCL<pcl::PointXYZ>;
template class omnimapper::OmniMapperVisualizerPCL<pcl::PointXYZRGBA>;
