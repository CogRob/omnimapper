#include <omnimapper_ros/error_evaluation_plugin.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/algorithm/string.hpp>

omnimapper::ErrorEvaluationPlugin::ErrorEvaluationPlugin (omnimapper::OmniMapperBase* mapper)
  : nh_ ("~"),
    marker_server_ (new interactive_markers::InteractiveMarkerServer ("OmniMapperError", "", false)),
    menu_handler_ (new interactive_markers::MenuHandler ()),
    mapper_ (mapper)
{
  marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray> ("/visualization_marker_array", 0);
  //marker_server_->clear ();

  // Create a control marker at the map origin for map level controls
  // visualization_msgs::InteractiveMarker origin_int_marker;
  // origin_int_marker.header.frame_id = "/world";
  // origin_int_marker.name = "OmniMapper";

  // visualization_msgs::Marker box_marker;
  // box_marker.type = visualization_msgs::Marker::CUBE;
  // box_marker.scale.x = 0.1;
  // box_marker.scale.y = 0.1;
  // box_marker.scale.z = 0.1;
  // box_marker.color.r = 0.5;
  // box_marker.color.g = 0.5;
  // box_marker.color.b = 0.5;
  // box_marker.color.a = 1.0;

  // visualization_msgs::InteractiveMarkerControl control;
  // control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
  // control.always_visible = true;
  // control.markers.push_back (box_marker);
  // origin_int_marker.controls.push_back (control);

  // marker_server_->insert (origin_int_marker);

  // initMenu ();

  // marker_server_->applyChanges ();
}

void
omnimapper::ErrorEvaluationPlugin::initMenu ()
{
  playback_menu_ = menu_handler_->insert ("Playback Control");
  interactive_markers::MenuHandler::EntryHandle play_pause = menu_handler_->insert (playback_menu_, "Play / Pause");
  menu_handler_->apply (*marker_server_, "OmniMapper");
  marker_server_->applyChanges ();
}

// void
// omnimapper::ErrorEvaluationPlugin::playPauseCb (const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
// {
//   printf ("Stopping playback!\n");
// }

void
omnimapper::ErrorEvaluationPlugin::poseClickCallback (const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback)
{
  // Parse the feedback
}

void
omnimapper::ErrorEvaluationPlugin::update (boost::shared_ptr<gtsam::Values>& vis_values, boost::shared_ptr<gtsam::NonlinearFactorGraph>& vis_graph)
{
  printf ("Updating Error evaluation plugin\n");
  gtsam::Values current_solution = *vis_values;
  gtsam::NonlinearFactorGraph current_graph = *vis_graph;

/*
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker mapper_graph;
  mapper_graph.header.frame_id = "/world";
  mapper_graph.header.stamp = ros::Time ();
  mapper_graph.ns = "mapper_trajectory";
  mapper_graph.id = 0;
  mapper_graph.type = visualization_msgs::Marker::LINE_LIST;
  mapper_graph.action = visualization_msgs::Marker::ADD;
  mapper_graph.color.a = 0.5;
  mapper_graph.color.r = 0.0;
  mapper_graph.color.g = 0.0;
  mapper_graph.color.b = 1.0;
  mapper_graph.scale.x = 0.01;

  visualization_msgs::Marker ground_truth_graph;
  ground_truth_graph.header.frame_id = "/world";
  ground_truth_graph.header.stamp = ros::Time ();
  ground_truth_graph.ns = "ground_truth_trajectory";
  ground_truth_graph.id = 0;
  ground_truth_graph.type = visualization_msgs::Marker::LINE_LIST;
  ground_truth_graph.action = visualization_msgs::Marker::ADD;
  ground_truth_graph.color.a = 0.5;
  ground_truth_graph.color.r = 0.0;
  ground_truth_graph.color.g = 1.0;
  ground_truth_graph.color.b = 0.0;
  ground_truth_graph.scale.x = 0.01;
  
  BOOST_FOREACH (const gtsam::NonlinearFactorGraph::sharedFactor& factor, current_graph)
  {
    // check for poses
    const std::vector<gtsam::Key> keys = factor->keys ();
    
    // skip if there aren't two pose keys
    if ((keys.size () == 2))
    {
      gtsam::Symbol sym1 (keys[0]);
      gtsam::Symbol sym2 (keys[1]);


      //if ((gtsam::symbolChr (keys[0]) == 'x') && (gtsam::symbolChr (keys[1]) == 'x'))
      if ((gtsam::symbolChr (keys[0]) == 'x') && (gtsam::symbolChr (keys[1]) == 'x') && (abs (sym1.index () - sym2.index ()) == 1))
      {
        // Draw the pose graph
        gtsam::Pose3 p1 = current_solution.at<gtsam::Pose3>(keys[0]);
        gtsam::Pose3 p2 = current_solution.at<gtsam::Pose3>(keys[1]);
        
        geometry_msgs::Point p1_msg;
        p1_msg.x = p1.x ();
        p1_msg.y = p1.y ();
        p1_msg.z = p1.z ();
        
        geometry_msgs::Point p2_msg;
        p2_msg.x = p2.x ();
        p2_msg.y = p2.y ();
        p2_msg.z = p2.z ();
        
        mapper_graph.points.push_back (p1_msg);
        mapper_graph.points.push_back (p2_msg);

        // Draw the ground truth version of the graph 
        omnimapper::Time t1;
        mapper_->getTimeAtPoseSymbol (sym1, t1);
        omnimapper::Time t2;
        mapper_->getTimeAtPoseSymbol (sym2, t2);
        
        std::cout << "T1: " << t1 << " T2: " << t2 << std::endl;

        gtsam::Pose3 gt_p1 = getPoseAtTime (t1);
        gtsam::Pose3 gt_p2 = getPoseAtTime (t2);
        
        geometry_msgs::Point gt_p1_msg;
        gt_p1_msg.x = gt_p1.x ();
        gt_p1_msg.y = gt_p1.y ();
        gt_p1_msg.z = gt_p1.z ();

        geometry_msgs::Point gt_p2_msg;
        gt_p2_msg.x = gt_p2.x ();
        gt_p2_msg.y = gt_p2.y ();
        gt_p2_msg.z = gt_p2.z ();

        ground_truth_graph.points.push_back (gt_p1_msg);
        ground_truth_graph.points.push_back (gt_p2_msg);
      }
    }
  }
  
  marker_array.markers.push_back (mapper_graph);
  marker_array.markers.push_back (ground_truth_graph);
  marker_array_pub_.publish (marker_array);

*/

  // Set up the error evaluation, for error modeling
  // double avg_sequential_position_error = 0.0;
  // double avg_sequential_angular_error = 0.0;
  // double max_sequential_position_error = 0.0;
  // double max_sequential_angular_error = 0.0;
  
  // BOOST_FOREACH (const gtsam::NonlinearFactorGraph::sharedFactor& factor, current_graph)
  // {
  //   // check for poses
  //   const std::vector<gtsam::Key> keys = factor->keys ();
    
  //   // skip if there aren't two pose keys
  //   if ((keys.size () == 2))
  //   {
  //     gtsam::Symbol sym1 (keys[0]);
  //     gtsam::Symbol sym2 (keys[1]);
      
  //     if ((gtsam::symbolChr (keys[0]) == 'x') && (gtsam::symbolChr (keys[1]) == 'x') && (abs (sym1.index () - sym2.index ()) == 1))
  //     {
  //     }
    
  //   }
  // }

  // Generate interactive markers for the poses
  // visualization_msgs::InteractiveMarker int_marker;
  // int_marker.header.frame_id = "/world";
  // int_marker.name = "mapper_trajectory";
  // int_marker.description = "Mapper Trajectory";
  
  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
  {
    geometry_msgs::Pose pose;    
    gtsam::Symbol key_symbol (key_value.key);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation ();
    int pose_idx = key_symbol.index ();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion ();

    omnimapper::Time t1;
    mapper_->getTimeAtPoseSymbol (key_symbol, t1);
    
    gtsam::Pose3 gt_p1 = getPoseAtTime (t1);
    
    // Make a sphere for the current pose
    visualization_msgs::Marker pose_marker;
    pose_marker.type = visualization_msgs::Marker::SPHERE;
    pose_marker.scale.x = 0.01;//0.005;
    pose_marker.scale.y = 0.01;//0.005;
    pose_marker.scale.z = 0.01;//0.005;
    pose_marker.color.r = 1.0;
    pose_marker.color.g = 0.0;
    pose_marker.color.b = 0.0;
    pose_marker.color.a = 1.0;
    pose_marker.pose.position.x = sam_pose.x ();
    pose_marker.pose.position.y = sam_pose.y ();
    pose_marker.pose.position.z = sam_pose.z ();
    pose_marker.pose.orientation.x = 0.0;
    pose_marker.pose.orientation.y = 0.0;
    pose_marker.pose.orientation.z = 0.0;
    pose_marker.pose.orientation.w = 1.0;

    // Ground Truth Marker
    std::string gt_name = std::string ("gt_") + std::string (key_symbol);
    visualization_msgs::InteractiveMarker gt_marker;
    bool gt_pose_exists = marker_server_->get (gt_name, gt_marker);
    
    if (!gt_pose_exists)
    {
      visualization_msgs::Marker gt_pose_marker;
      gt_pose_marker.type = visualization_msgs::Marker::SPHERE;
      gt_pose_marker.scale.x = 0.01;//0.005;
      gt_pose_marker.scale.y = 0.01;//0.005;
      gt_pose_marker.scale.z = 0.01;//0.005;
      gt_pose_marker.color.r = 0.0;
      gt_pose_marker.color.g = 1.0;
      gt_pose_marker.color.b = 0.0;
      gt_pose_marker.color.a = 1.0;
      gt_pose_marker.pose.position.x = gt_p1.x ();
      gt_pose_marker.pose.position.y = gt_p1.y ();
      gt_pose_marker.pose.position.z = gt_p1.z ();
      gt_pose_marker.pose.orientation.x = 0.0;
      gt_pose_marker.pose.orientation.y = 0.0;
      gt_pose_marker.pose.orientation.z = 0.0;
      gt_pose_marker.pose.orientation.w = 1.0;

      gt_marker.header.frame_id = "/world";
      gt_marker.header.stamp = ros::Time::now ();
      gt_marker.name = gt_name;

      visualization_msgs::InteractiveMarkerControl control;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      control.always_visible = true;
      control.markers.push_back (gt_pose_marker);
      gt_marker.controls.push_back (control);
      
      marker_server_->insert (gt_marker);

      // interactive_markers::MenuHandler::EntryHandle gt_menu = menu_handler_->insert (gt_name, boost::bind (&omnimapper::ErrorEvaluationPlugin::poseClickCallback, this, _1));
      // menu_handler_->apply (*marker_server_, gt_name);
      // marker_server_->applyChanges ();

    }

    // Try setting the pose first
    
    geometry_msgs::Pose origin_pose;
    std::string marker_name (key_symbol);
    std_msgs::Header marker_header;
    marker_header.frame_id = "/world";
    marker_header.stamp = ros::Time::now ();

    // visualization_msgs::InteractiveMarker current_marker;
    // bool test_get = marker_server_->get (marker_name, current_marker);

     bool set_pose_worked = marker_server_->setPose (marker_name, origin_pose, marker_header);
    //bool set_pose_worked = false;
    

    if (!set_pose_worked)
    {
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header = marker_header;//.frame_id = "/world";
      int_marker.name = marker_name;
      //int_marker.description = "Mapper Trajectory";

      // Make a button to hold the menu
      visualization_msgs::InteractiveMarkerControl control;
      control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
      control.always_visible = true;
      control.markers.push_back (pose_marker);
      int_marker.controls.push_back (control);
      
      marker_server_->insert (int_marker);
    }
    
  }
  marker_server_->applyChanges ();  

  //writeMapperTrajectoryFile (std::string (""), current_solution);

}

void
omnimapper::ErrorEvaluationPlugin::writeMapperTrajectoryFile (std::string trajectory_filename, gtsam::Values& current_solution)
{
  std::cout << "Writing Mapper Trajectory to: " << trajectory_filename << std::endl;
  //gtsam::Values current_solution = mapper_->getSolution ();
  std::ofstream trajectory_file (trajectory_filename.c_str ());

  gtsam::Values::ConstFiltered<gtsam::Pose3> pose_filtered = current_solution.filter<gtsam::Pose3>();
  BOOST_FOREACH (const gtsam::Values::ConstFiltered<gtsam::Pose3>::KeyValuePair& key_value, pose_filtered)
  {
    geometry_msgs::Pose pose;
    
    gtsam::Symbol key_symbol (key_value.key);
    omnimapper::Time key_time;
    mapper_->getTimeAtPoseSymbol (key_symbol, key_time);
    gtsam::Pose3 sam_pose = key_value.value;
    gtsam::Rot3 rot = sam_pose.rotation ();
    int pose_idx = key_symbol.index ();
    // W X Y Z
    gtsam::Vector quat = rot.quaternion ();
    boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration since_epoch = key_time - epoch;
    uint64_t seconds_part = since_epoch.total_seconds ();
    boost::posix_time::time_duration remaining_microseconds = since_epoch - boost::posix_time::seconds (since_epoch.total_seconds ());
    uint64_t microseconds_part = remaining_microseconds.total_microseconds ();

    trajectory_file <<
    boost::lexical_cast<std::string> (seconds_part) << "." << 
    boost::lexical_cast<std::string>(microseconds_part) << " " <<
    sam_pose.x () << " " <<
    sam_pose.y () << " " <<
    sam_pose.z () << " " <<
    quat[1] << " " <<
    quat[2] << " " << 
    quat[3] << " " <<
    quat[0] << " " << std::endl;
  }
  trajectory_file.close ();
}

void 
omnimapper::ErrorEvaluationPlugin::loadGroundTruthFile (std::string ground_truth_filename)
{
  // Load and parse ground truth file
  std::ifstream ground_truth_file (ground_truth_filename.c_str ());
  while (!ground_truth_file.eof ())
  {
    char buf[2048];
    ground_truth_file.getline (buf, 2048);
    
    // Skip comments
    if (buf[0] == '#')
      continue;
    
    if (!buf[0])
      continue;

    std::cout << "Processing line: " << buf << std::endl;

    std::string str (buf);
    std::vector<std::string> strs;
    boost::split (strs, str, boost::is_any_of ("\n "));
    std::string stamp_str = strs[0];
    std::vector<std::string> stamp_strs;
    boost::split (stamp_strs, stamp_str, boost::is_any_of ("."));
    long seconds = boost::lexical_cast<long> (stamp_strs[0]);
    // Note, while associated.txt uses microseconds, only 4 digits are provided in the ground truth file
    long microseconds = boost::lexical_cast<long> (stamp_strs[1]);
    microseconds *= 100;
    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::time_duration since_epoch = boost::posix_time::seconds (seconds) + boost::posix_time::microseconds (microseconds);
    boost::posix_time::ptime stamp_time = time_t_epoch + since_epoch;
    uint64_t stamp = since_epoch.total_microseconds ();   

    std::cout << "Time: " << stamp_time << std::endl;
    

    double tx = boost::lexical_cast<double>(strs[1]);
    double ty = boost::lexical_cast<double>(strs[2]);
    double tz = boost::lexical_cast<double>(strs[3]);
    double qx = boost::lexical_cast<double>(strs[4]);
    double qy = boost::lexical_cast<double>(strs[5]);
    double qz = boost::lexical_cast<double>(strs[6]);
    double qw = boost::lexical_cast<double>(strs[7]);
    
    gtsam::Pose3 pose (gtsam::Rot3::quaternion (qw, qx, qy, qz), gtsam::Point3 (tx, ty, tz));
    ground_truth_trajectory_.push_back (std::make_pair<omnimapper::Time, gtsam::Pose3> (stamp_time, pose));
  }
  printf ("OmniMapper Error Evaluation Plugin: Loaded %d ground truth trajectory poses.\n", ground_truth_trajectory_.size ());
}

void 
omnimapper::ErrorEvaluationPlugin::loadAssociatedFile (std::string associated_filename)
{
  std::ifstream associated_file (associated_filename.c_str ());
  while (!associated_file.eof ())
  {
    char buf[2048];
    associated_file.getline (buf, 2048);
    
    // Skip comments
    if (buf[0] == '#')
      continue;
      
    if (!buf[0])
      continue;

    std::vector<std::string> strs;
    std::string str (buf);
    boost::split (strs, str, boost::is_any_of ("\n "));
    std::string rgb_stamp_str = strs[0];
    std::string depth_stamp_str = strs[2];
    std::vector<std::string> depth_stamp_strs;
    boost::split (depth_stamp_strs, depth_stamp_str, boost::is_any_of ("."));
    long seconds = boost::lexical_cast<long>(depth_stamp_strs[0]);
    long microseconds = boost::lexical_cast<long>(depth_stamp_strs[1]);
    boost::posix_time::time_duration since_epoch = boost::posix_time::seconds (seconds) + boost::posix_time::microseconds (microseconds);
    boost::posix_time::ptime time_t_epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::ptime stamp_time = time_t_epoch + since_epoch;
    uint64_t stamp = since_epoch.total_microseconds ();
    cloud_timestamps_.push_back (stamp_time);
  }
  printf ("OmniMapper Error Evaluation Plugin: Loaded %d timestamps from associated file.\n", cloud_timestamps_.size ());
}

uint64_t 
omnimapper::ErrorEvaluationPlugin::getStampFromIndex (int idx)
{
  return (omnimapper::ptime2stamp (cloud_timestamps_[idx]));
}

gtsam::Pose3
omnimapper::ErrorEvaluationPlugin::getPoseAtTime (omnimapper::Time time)
{
  for (int i = 0; i < ground_truth_trajectory_.size () - 1; i++)
  {
    // If we've found a time interval that contains our requested time
    if ((ground_truth_trajectory_[i].first < time) && (time < ground_truth_trajectory_[i+1].first))
    {
      std::cout << "Ground Truth Pose Lookup: " << ground_truth_trajectory_[i].first << " < " << time << " < " << ground_truth_trajectory_[i+1].first << std::endl;
      // Check which is closer
      boost::posix_time::time_duration lower_diff = time - ground_truth_trajectory_[i].first;
      boost::posix_time::time_duration upper_diff = ground_truth_trajectory_[i+1].first - time;
      
      if (lower_diff < upper_diff)
        return (ground_truth_trajectory_[i].second);
      else
        return (ground_truth_trajectory_[i+1].second);
    }
  }
  printf ("OmniMapper Error Evaluation Plugin: Error, could not find timestamp within ground truth range!\n");
  assert (false);
}

gtsam::Pose3
omnimapper::ErrorEvaluationPlugin::getInitialPose ()
{
  // Lookup timestamp of 0th cloud
  return (getPoseAtTime (cloud_timestamps_[0]));
}
