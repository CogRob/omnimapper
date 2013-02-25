#include <omnimapper/omnimapper_base.h>

omnimapper::OmniMapperBase::OmniMapperBase ()
{
  debug_ = true;
  initialized_ = false;
  // TODO: make it optional to set an arbitrary initial pose
  //initializePose ();
  largest_pose_index = 0;
  latest_committed_node = chain.begin ();
  latest_commit_time = boost::posix_time::microsec_clock::local_time();
  commit_window = 1.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::initializePose (Time& t)
{
  // TODO: make this a default param, but changeable
  gtsam::Symbol init_symbol ('x', 0);
  current_pose_symbol = init_symbol;
  gtsam::Pose3 init_pose = gtsam::Pose3 (gtsam::Rot3::ypr (0.0, 0.0, 0.0), gtsam::Point3 (0.0, 0.0, 0.0));
  new_values.insert (init_symbol, init_pose);
  gtsam::PriorFactor<gtsam::Pose3> posePrior (init_symbol, init_pose, gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)));
  new_factors.add (posePrior);
  PoseChainNode init_node (t, init_symbol);
  init_node.status = omnimapper::PoseChainNode::COMMITTED;
  //chain.push_back (init_node);
  std::list<omnimapper::PoseChainNode>::iterator new_itr = chain.insert (chain.end (), init_node);
  
  symbol_lookup.insert (std::pair<gtsam::Symbol, std::list<omnimapper::PoseChainNode>::iterator>(init_symbol, new_itr));
  time_lookup.insert (std::pair<Time, std::list<omnimapper::PoseChainNode>::iterator>(t, new_itr));
  initialized_ = true;
  latest_committed_node = new_itr;
  printf ("OmnimapperBase: Initialized!\n");
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::commitNextPoseNode ()
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  if (debug_)
  {  
    printf ("chain size: %d\n", chain.size ());
    for (std::list<omnimapper::PoseChainNode>::iterator itr = chain.begin (); itr != chain.end (); itr++)
    {
      printf ("node: %c %d %u %d\n", itr->symbol.chr (), itr->symbol.index (), itr->time, itr->factors.size ());
    }
  }
  
  
  // Do nothing if there are no new factors
  std::list<omnimapper::PoseChainNode>::iterator to_commit = latest_committed_node;
  to_commit++;
  if (debug_)
  {
    printf ("latest: %c %d\n", latest_committed_node->symbol.chr (), latest_committed_node->symbol.index ());
    printf ("to commit: %c %d\n", to_commit->symbol.chr (), to_commit->symbol.index ());
  }

  if (to_commit == chain.end ())
  {
    printf ("OmniMapper: commitNext called when latest node has already been committed!\n");
    return;
  }
  
  // Check that enough time has elapsed
  if (!((boost::posix_time::microsec_clock::local_time() - to_commit->time) > boost::posix_time::seconds (commit_window)))
  {
    printf ("OmniMapper: commitNext -- not time to commit yet!\n");
    return;
  }

  if (debug_)
    printf ("OmniMapper: Commiting...\n");
  // TODO: verify that this won't break our chain by inspecting pose information
  // Call pose plugins to add pose factors between previous chain time and current pose time
  bool initialized = false;
  for (int i = 0; i < pose_plugins.size (); i++)
  {
    // TODO: make this a boost::optional, in case the plugin is disabled or unable to give a pose
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr new_pose_factor = pose_plugins[i]->addRelativePose (latest_committed_node->time, latest_committed_node->symbol, to_commit->time, to_commit->symbol);
    //addFactor (new_pose_factor);
    new_factors.push_back (new_pose_factor);
    // Initialize the value if we're the first one
    if (!initialized)//TODO: check new_pose_factor isn't boost::none
    {
      gtsam::Pose3 relative_pose = new_pose_factor->measured ();
      // Compose the relative pose with the previously optimized pose
      gtsam::Pose3 new_pose_value = current_solution.at<gtsam::Pose3> (latest_committed_node->symbol).compose (relative_pose);
      new_values.insert (to_commit->symbol, new_pose_value);
      initialized = true;
    }
  }
  // Add the rest of the factors
  if (!initialized)
  {
    // If we have no pose factors, we need a relative pose measurement to initialize the pose
    for (int i = 0; i < to_commit->factors.size (); i++)
    {
      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between = boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >((to_commit->factors[i]));
      if (between != NULL)
      {
        gtsam::Pose3 relative_pose = between->measured ();
        gtsam::Pose3 new_pose_value = current_solution.at<gtsam::Pose3> (latest_committed_node->symbol).compose (relative_pose);
        new_values.insert (to_commit->symbol, new_pose_value);
        initialized = true;
        break;
      }
    }
    
    // If we didn't find any, we can't commit yet!
    if (!initialized)
    {
      if (debug_)
      {
        printf ("OmniMapper: Tried to commit without any between factors!  Waiting for between factor!\n");
        printf ("Node has %d factors\n", to_commit->factors.size ());
      }
      return;
    }
  }
  new_factors.push_back (to_commit->factors);
  to_commit->status = omnimapper::PoseChainNode::COMMITTED;
  to_commit->factors.clear ();
  latest_committed_node++;
  latest_commit_time = boost::posix_time::microsec_clock::local_time();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
omnimapper::OmniMapperBase::addFactorDirect (gtsam::NonlinearFactor::shared_ptr& new_factor)
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  new_factors.push_back (new_factor);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
omnimapper::OmniMapperBase::addFactor (gtsam::NonlinearFactor::shared_ptr& new_factor)
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  // Find the pose keys related to this factor, adding this to the latest one
  const std::vector<gtsam::Key> keys = new_factor->keys ();
  int latest_pose_idx = -1;
  Time latest_pose_time = boost::posix_time::neg_infin; // Probably nobody has data earlier than this?
  printf ("addFactor: starting to look at keys\n");
  for (int i = 0; i < keys.size (); i++)
  {
    if (gtsam::symbolChr (keys[i]) == 'x')
    {
      printf ("Going to compare keys\n");
      if (symbol_lookup[keys[i]]->time > latest_pose_time)
      {
        latest_pose_idx = i;
        latest_pose_time = symbol_lookup[keys[i]]->time ;
      }
    }
  }
  
  // Add this factor to the pose chain at the latest pose
  // TODO: handle factors unrelated to any pose.  We can probably just go ahead and add them directly
  if (latest_pose_idx == -1)
  {
    printf ("OmniMapper: Error - no pose factor associated with this factor.  Not yet supported!\n");
    return false;
  }
  
  symbol_lookup[keys[latest_pose_idx]]->symbol.print ("OmniMapper: Added factor to pose: ");

  // If that pose has been committed already, we can add this directly for the next optimization run.
  if (symbol_lookup[keys[latest_pose_idx]]->status == omnimapper::PoseChainNode::COMMITTED)
  {
    new_factors.push_back (new_factor);
  }
  else
  {
    // If it hasn't yet been commited, we should add this to the pending factors, causing pose factors to add constraints for this timestamp
    symbol_lookup[keys[latest_pose_idx]]->factors.push_back (new_factor);
  }
  
  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::getPoseSymbolAtTime (Time& t, gtsam::Symbol& sym)
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  // If we haven't initialized yet, we do so on the first symbol request
  if (!initialized_)
  {
    initializePose (t);
    optimize ();
  }

  if (time_lookup.count (t) > 0)
  {
    // If we have a pose, just return it
    sym = time_lookup[t]->symbol;
    printf ("OmniMapperBase: We have this time already, returning it\n");
    return;
  }
  else
  {
    // If we don't have a pose yet, make one
    printf ("OmniMapperBase: Making a new entry\n");
    largest_pose_index++;
    gtsam::Symbol new_sym ('x', largest_pose_index);
    sym = new_sym;
    // Add a relevant node to the chain
    omnimapper::PoseChainNode new_node (t, new_sym);
    // TODO: replace this with an STL search operation
    for (std::list<omnimapper::PoseChainNode>::iterator itr = chain.end (); itr != chain.begin (); --itr)
    {
      if (itr->time  < t)
      {
        //++itr;
        std::list<omnimapper::PoseChainNode>::iterator new_itr = chain.insert (itr, new_node);
        time_lookup.insert (std::pair<Time, std::list<omnimapper::PoseChainNode>::iterator>(t, new_itr));
        symbol_lookup.insert (std::pair<gtsam::Symbol, std::list<omnimapper::PoseChainNode>::iterator>(new_sym, new_itr));
        break;
      }
    }
    return;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gtsam::Values 
omnimapper::OmniMapperBase::getSolution ()
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  return (current_solution);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::printSolution ()
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  current_solution.print ("Current OmniMapper Solution: \n");
  current_graph.print ("Current OmniMapper Graph: \n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::optimize ()
{
  printf ("OmniMapper: optimizing with:\n");
  new_factors.print ("New Factors: ");
  new_values.print ("New Values: ");
  gtsam::ISAM2Result result = isam2.update (new_factors, new_values);
  current_solution = isam2.calculateEstimate ();
  current_graph = isam2.getFactorsUnsafe (); // TODO: is this necessary and okay?
  new_factors = gtsam::NonlinearFactorGraph ();
  new_values.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//void
//omnimapper::OmniMapperBase::addMeasurementPlugin (omnimapper::MeasurementPlugin& plugin)
//{
//  measurement_plugins.push_back (plugin);
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::addPosePlugin (omnimapper::OmniMapperBase::PosePluginPtr& plugin)
{
  pose_plugins.push_back (plugin);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
omnimapper::OmniMapperBase::addOutputPlugin (omnimapper::OmniMapperBase::OutputPluginPtr& plugin)
{
  output_plugins.push_back (plugin);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// bool
// omnimapper::OmniMapperBase::addFactor (NonlinearFactorPtr& new_factor)
// {
//   new_factors.push_back (new_factor);
//   return (true);
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
omnimapper::OmniMapperBase::addNewValue (gtsam::Symbol& new_symbol, gtsam::Value& new_value)
{
  new_values.insert (new_symbol, new_value);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::optional<gtsam::Pose3>
omnimapper::OmniMapperBase::getPose (gtsam::Symbol& pose_sym)
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  if (current_solution.exists<gtsam::Pose3>(pose_sym))
    return (current_solution.at<gtsam::Pose3> (pose_sym));
  else
    return (boost::none);
}

//------------------//------------------//------------------//------------------//------------------//------------------
//------------------//------------------//------------------//------------------//------------------//------------------
//------------------//------------------//------------------//------------------//------------------//------------------
//------------------//------------------//------------------//------------------//------------------//------------------

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// bool
// omnimapper::OmniMapperBase::shouldAddPose ()
// {
//   if (shouldAddPoseFn == NULL)
//   {
//     //if (debug_)
//     //  printf ("OmniMapperBase: using default shouldAddPose\n");
//     return true;
//   }
//   else
//   {
//     //if (debug_)
//     //  printf ("OmniMapperBase: using user specified shouldAddPose\n");
//     return (shouldAddPoseFn ());
//   }
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void
// omnimapper::OmniMapperBase::spinOnce ()
// {
//   printf ("spinning...\n");
//   // Check if we should add a new pose
//   if (shouldAddPose ())
//   {
//     printf ("appending pose...\n");
//     // Create a new pose and add pose factors
//     gtsam::Symbol new_pose_symbol = appendPose ();
//     // Add Feature Measurements (if applicable)
//     addMeasurements ();
//     // Re-optimize with this new information
//     optimize ();
//     // Update Visualization with new information
//     boost::shared_ptr<gtsam::Values> vis_values (new gtsam::Values ());
//     // Make a copy, to ensure we don't have
//     *vis_values = getSolution ();
//     for (int i = 0; i < output_plugins.size (); i++)
//     {
//       output_plugins[i]->update (vis_values);
//     }
//   }
// }

void
omnimapper::OmniMapperBase::spin ()
{
  while (true)
  {
    spinOnce ();
    boost::this_thread::sleep (boost::posix_time::milliseconds (100));
  }
}

void
omnimapper::OmniMapperBase::spinOnce ()
{
  printf ("OMB: spinning...\n");
  // If time to commit
  //std::list<omnimapper::PoseChainNode>::iterator next_node = latest_committed_node;
  //next_node++;
  //if (boost::posix_time::microsec_clock::local_time() - latest_commit_time > boost::posix_time::seconds (commit_window))
  commitNextPoseNode ();
  if (new_factors.size () > 0)
  {
    printf ("OMB: optimizing\n");
    optimize ();
    updateOutputPlugins ();
    // Print latest
    gtsam::Pose3 new_pose_value = current_solution.at<gtsam::Pose3> (latest_committed_node->symbol);
    printf ("OMB Latest Pose: %lf %lf %lf\n", new_pose_value.x (), new_pose_value.y (), new_pose_value.z ());
  } 
  else 
  {
    printf ("OMB: No new factors to optimize\n");
  }
  
}

void
omnimapper::OmniMapperBase::updateOutputPlugins ()
{
  boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::shared_ptr<gtsam::Values> vis_values (new gtsam::Values (current_solution));
  for (int i = 0; i < output_plugins.size (); i++)
  {
    printf ("Updating plugin %d with %d values\n", i, vis_values->size ());
    output_plugins[i]->update (vis_values);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*void
omnimapper::OmniMapperBase::spinOnce ()
{
  printf ("spinning...\n");
  // Check if we should add a new pose
  if (shouldAddPose ())
  {
    printf ("appending pose...\n");
    // Create a new pose and add pose factors
    gtsam::Symbol new_pose_symbol = appendPose ();
    // Add Feature Measurements (if applicable)
    addMeasurements ();
    // Re-optimize with this new information
    optimize ();
    // Update Visualization with new information
    boost::shared_ptr<gtsam::Values> vis_values (new gtsam::Values ());
    // Make a copy, to ensure we don't have
    *vis_values = getSolution ();
    for (int i = 0; i < output_plugins.size (); i++)
    {
      output_plugins[i]->update (vis_values);
    }
  }
  }*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// gtsam::Symbol
// omnimapper::OmniMapperBase::appendPose ()
// {
//   // Ensure that we have at least one pose plugin
//   if (pose_plugins.size () < 1)
//   {
//     printf ("OmniMapper: Error!  At least one pose plugin is required.\n");
//   }

//   // Create a new pose and add factors for the new plugin
//   //if (current_pose_symbol.index () != 0)
//   //{
//     gtsam::Symbol new_symbol ('x', current_pose_symbol.index ()+1);
//     current_pose_symbol = new_symbol;
//     //}
//   printf ("Appending pose with pose %d\n", current_pose_symbol.index ());
  
//   pose_plugins[0]->appendPose (true);
//   // Add new pose factors for each subsequent pose plugin
//   for (int i = 1; i < pose_plugins.size (); i++)
//   {
//     pose_plugins[i]->appendPose ();
//   }
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void
// omnimapper::OmniMapperBase::getPoseSymbolAtTime (Time& t, gtsam::Symbol& sym, bool& needs_initialization)
// {
//   // See if we have one already
//   if (symbol_times.count (t) > 0)
//   {
//     sym = symbol_times[t];
//     needs_initialization = false;
//     return;
//   }
//   else if (symbol_times.size () == 0)
//   {
//     // If this is the first pose
//     initializePose (t);
//     sym = gtsam::Symbol ('x', 0);
//     needs_initialization = false;
//     return;
//   }
//   else
//   {
//     // Create a new symbol
//     largest_pose_index++;
//     gtsam::Symbol new_symbol ('x', largest_pose_index);
//     symbol_times.insert (std::pair<Time, gtsam::Symbol>(t, new_symbol));
//     sym = new_symbol;
//     needs_initialization = true;
//     return;
//   }
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// bool
// omnimapper::OmniMapperBase::addRelativePoseMeasurement (Time t1, Time t2, const gtsam::Pose3& relative_pose, const gtsam::SharedNoiseModel& noise_model)
// {
//   // TODO: check movement threshold
//   // Find pose symbol at t1
//   gtsam::Symbol t1_sym;
//   bool needs_init;
//   getPoseSymbolAtTime (t1, t1_sym,);
//   // Find pose symbol at t2
//   gtsam::Symbol t2_sym = getPoseSymbolAtTime (t2);
//   // Create Between factor
//   omnimapper::OmniMapperBase::NonlinearFactorPtr between (new gtsam::BetweenFactor<gtsam::Pose3> (t1_sym, t2_sym, relative_pose, noise_model));
//   // Add it
//   addFactor (between);
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void
// omnimapper::OmniMapperBase::setShouldAddPoseFn (boost::function<bool()> fn)
// {
//   shouldAddPoseFn = fn;
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void
// omnimapper::OmniMapperBase::initializePlugins ()
// {
//   bool initialized = false;
//   while (!initialized)
//   {
//     bool found_not_ready = false;
//     for (int i  = 0; i < pose_plugins.size (); i ++)
//     {
//       bool ready = pose_plugins[i]->addInitialPose ();
//       if (!ready)
//         found_not_ready = true;
//       printf ("plugin %d: %d\n", i, ready);
//     }
//     if (!found_not_ready)
//       initialized = true;
//   }
//   // Call optimize to put this in the current solution.
//   optimize ();

//   printf ("OmniMapperBase: Plugin Initialization Complete.\n");
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void
// omnimapper::OmniMapperBase::addMeasurements ()
// {
//   // If we have no measurement plugins, do nothing
//   if (measurement_plugins.size () == 0)
//     return;
//   else
//     printf ("OmniMapper: addMeasurments is NYI!");
// }
