#include <omnimapper/omnimapper_base.h>
#include <pcl/common/time.h>  //TODO: remove, debug only

omnimapper::OmniMapperBase::OmniMapperBase()
    : initialized_(false),
      initial_pose_(gtsam::Pose3::identity()),
      get_time_(new GetSystemTimeFunctor()) {
  debug_ = true;
  // TODO: make it optional to set an arbitrary initial pose
  // initializePose ();
  suppress_commit_window_ = false;
  largest_pose_index = 0;
  latest_committed_node = chain.begin();
  latest_commit_time =
      (*get_time_)();  // boost::posix_time::microsec_clock::local_time();
  commit_window = 3.0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::initializePose(Time& t) {
  // boost::mutex::scoped_lock (omnimapper_mutex_);

  if (initialized_) {
    printf(
        "OmniMapperBase: ERROR!  CALLED INITIALIZE WHEN ALREADY "
        "INITIALIZED!\n");
    return;
  }

  // TODO: make this a default param, but changeable
  gtsam::Symbol init_symbol('x', 0);
  current_pose_symbol = init_symbol;
  // gtsam::Pose3 init_pose = gtsam::Pose3 (gtsam::Rot3::ypr (0.0, 0.0, 0.0),
  // gtsam::Point3 (0.0, 0.0, 0.0)); new_values.insert (init_symbol, init_pose);
  new_values.insert(init_symbol, initial_pose_);
  // gtsam::PriorFactor<gtsam::Pose3> posePrior (init_symbol, init_pose,
  // gtsam::noiseModel::Diagonal::Sigmas (gtsam::Vector_ (6, 0.001, 0.001, 0.001,
  // 0.001, 0.001, 0.001)));
  gtsam::PriorFactor<gtsam::Pose3> posePrior(
      init_symbol, initial_pose_,
      gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector(6) << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001)));
  new_factors.add(posePrior);
  PoseChainNode init_node(t, init_symbol);
  init_node.status = omnimapper::PoseChainNode::COMMITTED;
  // chain.push_back (init_node);
  std::list<omnimapper::PoseChainNode>::iterator new_itr =
      chain.insert(chain.end(), init_node);

  symbol_lookup.insert(
      std::pair<gtsam::Symbol, std::list<omnimapper::PoseChainNode>::iterator>(
          init_symbol, new_itr));
  time_lookup.insert(
      std::pair<Time, std::list<omnimapper::PoseChainNode>::iterator>(t,
                                                                      new_itr));
  // optimize ();

  gtsam::ISAM2Result result = isam2.update(new_factors, new_values);
  current_solution = isam2.calculateEstimate();
  // current_graph = isam2.getFactorsUnsafe (); // TODO: is this necessary and
  // okay?
  new_factors = gtsam::NonlinearFactorGraph();
  new_values.clear();

  initialized_ = true;
  latest_committed_node = new_itr;
  printf("OmnimapperBase: Initialized!\n");
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::setInitialPose(gtsam::Pose3& initial_pose) {
  initial_pose_ = initial_pose;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::setTimeFunctor(
    omnimapper::GetTimeFunctorPtr time_functor) {
  get_time_ = time_functor;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omnimapper::OmniMapperBase::commitNextPoseNode() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  if (debug_ && false) {
    printf("chain size: %zu\n", chain.size());
    for (std::list<omnimapper::PoseChainNode>::iterator itr = chain.begin();
         itr != chain.end(); itr++) {
      printf("node: %c %zu %u %zu\n", itr->symbol.chr(), itr->symbol.index(),
             itr->time, itr->factors.size());
      std::cout << "stamp: " << itr->time << std::endl;
    }
  }

  // Do nothing if there are no new factors
  std::list<omnimapper::PoseChainNode>::iterator to_commit =
      latest_committed_node;
  to_commit++;
  if (debug_) {
    printf("latest: %c %zu\n", latest_committed_node->symbol.chr(),
           latest_committed_node->symbol.index());
    printf("to commit: %c %zu\n", to_commit->symbol.chr(),
           to_commit->symbol.index());
  }

  if (to_commit == chain.end()) {
    if (debug_)
      printf(
          "OmniMapper: commitNext called when latest node has already been "
          "committed!\n");
    return (false);
  }

  // Check that enough time has elapsed
  if (!suppress_commit_window_) {
    // if (!((boost::posix_time::microsec_clock::local_time() - to_commit->time)
    // > boost::posix_time::seconds (commit_window)))
    if (!(((*get_time_)() - to_commit->time) >
          boost::posix_time::seconds(commit_window))) {
      printf("OmniMapper: commitNext -- not time to commit yet!\n");
      return (false);
    }
  }

  if (debug_) printf("OmniMapper: Commiting...\n");
  // TODO: verify that this won't break our chain by inspecting pose information
  // Call pose plugins to add pose factors between previous chain time and
  // current pose time
  bool initialized = false;

  for (int i = 0; i < pose_plugins.size(); i++) {
    // TODO: make this a boost::optional, in case the plugin is disabled or
    // unable to give a pose
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr new_pose_factor =
        pose_plugins[i]->addRelativePose(latest_committed_node->time,
                                         latest_committed_node->symbol,
                                         to_commit->time, to_commit->symbol);
    // addFactor (new_pose_factor);
    new_factors.push_back(new_pose_factor);
    // Initialize the value if we're the first one
    if (!initialized)  // TODO: check new_pose_factor isn't boost::none
    {
      gtsam::Pose3 relative_pose = new_pose_factor->measured();
      // Compose the relative pose with the previously optimized pose
      gtsam::Pose3 new_pose_value =
          current_solution.at<gtsam::Pose3>(latest_committed_node->symbol)
              .compose(relative_pose);

      // Note: this is a workaround for a GTSAM bug, where compose (also
      // between) return invalid/nan poses for small motions
      if (!(std::isfinite(new_pose_value.x()) &&
            std::isfinite(new_pose_value.y()) &&
            std::isfinite(new_pose_value.z()) &&
            std::isfinite(new_pose_value.rotation().matrix().determinant()))) {
        new_pose_value =
            current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);
      }

      new_values.insert(to_commit->symbol, new_pose_value);
      initialized = true;
    }
  }

  // Add the rest of the factors
  if (!initialized) {
    // If we have no pose factors, we need a relative pose measurement to
    // initialize the pose
    for (int i = 0; i < to_commit->factors.size(); i++) {
      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
              (to_commit->factors[i]));
      if (between != NULL) {
        gtsam::Pose3 relative_pose = between->measured();
        gtsam::Pose3 new_pose_value =
            current_solution.at<gtsam::Pose3>(latest_committed_node->symbol)
                .compose(relative_pose);
        // Note: this is a workaround for a GTSAM bug, where compose (also
        // between) return invalid/nan poses for small motions
        if (!(std::isfinite(new_pose_value.x()) &&
              std::isfinite(new_pose_value.y()) &&
              std::isfinite(new_pose_value.z()) &&
              std::isfinite(
                  new_pose_value.rotation().matrix().determinant()))) {
          new_pose_value =
              current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);
        }

        new_values.insert(to_commit->symbol, new_pose_value);
        initialized = true;
        break;
      }
    }

    // If we didn't find any, we can't commit yet!
    if (!initialized) {
      if (debug_) {
        printf(
            "OmniMapper: Tried to commit without any between factors!  Waiting "
            "for between factor!\n");
        printf("Node has %zu factors\n", to_commit->factors.size());
      }
      return (false);
    }
  }

  // Commit
  // printf ("Committing!\n");
  new_factors.push_back(to_commit->factors);
  to_commit->status = omnimapper::PoseChainNode::COMMITTED;
  to_commit->factors.clear();
  latest_committed_node++;
  latest_commit_time =
      (*get_time_)();  // boost::posix_time::microsec_clock::local_time();
  // printf ("Committed!\n");

  if (debug_) {
    new_factors.print("New Factors: \n");
    new_values.print("New Values: \n");
  }

  // Optimize
  // printf ("Optimizing!\n");
  gtsam::ISAM2Result result = isam2.update(new_factors, new_values);
  current_solution = isam2.calculateEstimate();
  current_graph =
      isam2.getFactorsUnsafe();  // TODO: is this necessary and okay?
  new_factors = gtsam::NonlinearFactorGraph();
  new_values.clear();
  // printf ("Optimized!\n");
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omnimapper::OmniMapperBase::addFactorDirect(
    gtsam::NonlinearFactor::shared_ptr& new_factor) {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  new_factors.push_back(new_factor);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omnimapper::OmniMapperBase::addFactor(
    gtsam::NonlinearFactor::shared_ptr& new_factor) {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  // Find the pose keys related to this factor, adding this to the latest one
  const std::vector<gtsam::Key> keys = new_factor->keys();

  int latest_pose_idx = -1;
  Time latest_pose_time =
      boost::posix_time::neg_infin;  // Probably nobody has data earlier than
                                     // this?
  if (debug_) printf("addFactor: starting to look at keys\n");
  for (int i = 0; i < keys.size(); i++) {
    if (gtsam::symbolChr(keys[i]) == 'x') {
      if (debug_) printf("Going to compare keys\n");
      if (symbol_lookup[keys[i]]->time > latest_pose_time) {
        latest_pose_idx = i;
        latest_pose_time = symbol_lookup[keys[i]]->time;
      }
    } else if (gtsam::symbolChr(keys[i]) == 'o' &&
               i == 0)  // TODO:@atrevor -- why is this here
    {
      if (debug_) std::cout << "Adding object between factor" << std::endl;
      new_factors.push_back(new_factor);
    }
  }

  // Add this factor to the pose chain at the latest pose
  // TODO: handle factors unrelated to any pose.  We can probably just go ahead
  // and add them directly
  if (latest_pose_idx == -1) {
    printf(
        "OmniMapper: Error - no pose factor associated with this factor.  Not "
        "yet supported!\n");
    return false;
  }

  if (debug_)
    symbol_lookup[keys[latest_pose_idx]]->symbol.print(
        "OmniMapper: Added factor to pose: ");

  // If that pose has been committed already, we can add this directly for the
  // next optimization run.
  if (symbol_lookup[keys[latest_pose_idx]]->status ==
      omnimapper::PoseChainNode::COMMITTED) {
    if (debug_) printf("About to push back new factor directly\n");
    new_factors.push_back(new_factor);
    if (debug_) printf("New factor pushed back direclty\n");
  } else {
    // If it hasn't yet been commited, we should add this to the pending
    // factors, causing pose factors to add constraints for this timestamp
    if (debug_) printf("Adding this to the pending factors\n");
    symbol_lookup[keys[latest_pose_idx]]->factors.push_back(new_factor);
    if (debug_) printf("Added to the pending factors!\n");
  }

  return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::getPoseSymbolAtTime(Time& t,
                                                     gtsam::Symbol& sym) {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  // If we haven't initialized yet, we do so on the first symbol request
  if (!initialized_) {
    if (debug_) printf("got symbol prior to initializing!\n");
    initializePose(t);
    if (debug_) printf("done initializing!\n");
  }

  // If we have a pose symbol for this timestamp, just return it
  if (time_lookup.count(t) > 0) {
    sym = time_lookup[t]->symbol;
    printf("OmniMapperBase: We have this time already, returning it\n");
    return;
  } else {
    // If we don't have a pose yet, make one
    if (debug_) printf("OmniMapperBase: Making a new entry\n");
    // Check that the time is after the latest committed pose time, otherwise
    // we'd need to splice it into the pose chain, which is not yet supported
    if (t < latest_committed_node->time) {
      std::cout
          << "OmniMapperBase: ERROR: Pose symbol requested for timestamp "
             "earlier than latest committed stamp! Increase the commit window "
             "length, or implement pose chain splicing (not yet implemented)."
          << std::endl;
      std::cout << "OmniMapperBase: requested time: " << t << std::endl;
      std::cout << "OmniMapperBase: latest committed time: "
                << latest_committed_node->time << std::endl;
      assert(false);
    }

    largest_pose_index++;
    gtsam::Symbol new_sym('x', largest_pose_index);
    sym = new_sym;
    // Add a relevant node to the chain
    omnimapper::PoseChainNode new_node(t, new_sym);
    if (debug_)
      std::cout << "OmniMapperBase: need new symbol at: " << t << std::endl;
    // TODO: replace this with an STL search operation
    // for (std::list<omnimapper::PoseChainNode>::iterator itr = chain.end ();
    // itr != chain.begin (); --itr)
    // {
    //   if (itr->time < t)
    //   {
    //     //++itr;
    //     //if (itr == chain.end ())
    //     //  std::cout << "inserting at end of chain!" << std::endl;
    //     //else
    //       std::cout << "inserting prior to timestamp: " << itr->time <<
    //       std::endl;
    //     std::list<omnimapper::PoseChainNode>::iterator new_itr = chain.insert
    //     (itr, new_node); time_lookup.insert (std::pair<Time,
    //     std::list<omnimapper::PoseChainNode>::iterator>(t, new_itr));
    //     symbol_lookup.insert (std::pair<gtsam::Symbol,
    //     std::list<omnimapper::PoseChainNode>::iterator>(new_sym, new_itr));
    //     break;
    //   }
    // }

    // Find the node with the previous stamp
    std::list<omnimapper::PoseChainNode>::iterator itr = chain.end();
    itr--;
    for (; itr != chain.begin(); itr--) {
      if (itr->time < t) break;
    }
    itr++;

    std::list<omnimapper::PoseChainNode>::iterator new_itr =
        chain.insert(itr, new_node);
    time_lookup.insert(
        std::pair<Time, std::list<omnimapper::PoseChainNode>::iterator>(
            t, new_itr));
    symbol_lookup.insert(
        std::pair<gtsam::Symbol,
                  std::list<omnimapper::PoseChainNode>::iterator>(new_sym,
                                                                  new_itr));
    return;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::getTimeAtPoseSymbol(gtsam::Symbol& sym,
                                                     Time& t) {
  // boost::lock_guard<boost::mutex> lock (omnimapper_mutex_);
  t = symbol_lookup[sym]->time;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gtsam::NonlinearFactorGraph omnimapper::OmniMapperBase::getGraph() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  return (current_graph);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gtsam::NonlinearFactorGraph
omnimapper::OmniMapperBase::getGraphAndUncommitted() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  gtsam::NonlinearFactorGraph graph = current_graph;
  graph.push_back(new_factors.begin(), new_factors.end());
  return (graph);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gtsam::Values omnimapper::OmniMapperBase::getSolution() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  return (current_solution);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
gtsam::Values omnimapper::OmniMapperBase::getSolutionAndUncommitted() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  // Start with the  initial solution
  gtsam::Values solution = current_solution;

  // Now add all landmarks from the new values
  solution.insert(new_values);

  return (solution);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::printSolution() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  current_solution.print("Current OmniMapper Solution: \n");
  current_graph.print("Current OmniMapper Graph: \n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::optimize() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  double opt_start = pcl::getTime();
  if (debug_) {
    current_solution.print("Current Solution: ");
    printf("OmniMapper: optimizing with:\n");
    new_factors.print("New Factors: ");
    new_values.print("New Values: ");
  }

  gtsam::ISAM2Result result = isam2.update(new_factors, new_values);
  current_solution = isam2.calculateEstimate();
  current_graph =
      isam2.getFactorsUnsafe();  // TODO: is this necessary and okay?
  new_factors = gtsam::NonlinearFactorGraph();
  new_values.clear();
  double opt_end = pcl::getTime();
  if (debug_)
    std::cout << "OmniMapperBase: optimize() took: "
              << double(opt_end - opt_start) << std::endl;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// void
// omnimapper::OmniMapperBase::addMeasurementPlugin
// (omnimapper::MeasurementPlugin& plugin)
//{
//  measurement_plugins.push_back (plugin);
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::addPosePlugin(
    omnimapper::OmniMapperBase::PosePluginPtr& plugin) {
  pose_plugins.push_back(plugin);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::addOutputPlugin(
    omnimapper::OmniMapperBase::OutputPluginPtr& plugin) {
  output_plugins.push_back(plugin);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// bool
// omnimapper::OmniMapperBase::addFactor (NonlinearFactorPtr& new_factor)
// {
//   new_factors.push_back (new_factor);
//   return (true);
// }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool omnimapper::OmniMapperBase::addNewValue(gtsam::Symbol& new_symbol,
                                             gtsam::Value& new_value) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  std::cout << "Adding new symbol: " << new_symbol << std::endl;
  new_values.insert(new_symbol, new_value);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void omnimapper::OmniMapperBase::updateValue(gtsam::Symbol& update_symbol,
                                             gtsam::Value& update_value) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  std::cout << "Updating symbol: " << update_symbol << std::endl;
  // Check new values first
  if (new_values.exists(update_symbol)) {
    new_values.update(update_symbol, update_value);
    return;
  }
  std::cout << "Update value not supported!" << std::endl;
  assert(false);
  exit(1);
  // gtsam::Values& state = isam2.getLinearizationPointUnsafe ();
  // state.update (update_symbol, update_value);
  return;
}

void omnimapper::OmniMapperBase::updatePlane(gtsam::Symbol& update_symbol,
                                             gtsam::Pose3& pose,
                                             gtsam::Plane<PointT>& meas_plane) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (new_values.exists(update_symbol)) {
    gtsam::Plane<PointT> to_update =
        new_values.at<gtsam::Plane<PointT> >(update_symbol);
    to_update.Extend2(pose, meas_plane);
    new_values.update(update_symbol, to_update);
    return;
  }

  std::cout << "Update value not supported!" << std::endl;
  assert(false);
  exit(1);
  // gtsam::Values& state = isam2.getLinearizationPointUnsafe ();
  // gtsam::Plane<PointT> to_update = state.at<gtsam::Plane<PointT>
  // >(update_symbol); to_update.Extend2 (pose, meas_plane);
  /////to_update.Extend (pose, meas_plane);

  // state.update (update_symbol, to_update);
  return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::optional<gtsam::Pose3> omnimapper::OmniMapperBase::getPose(
    gtsam::Symbol& pose_sym) {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (current_solution.exists<gtsam::Pose3>(pose_sym))
    return (current_solution.at<gtsam::Pose3>(pose_sym));
  else
    return (boost::none);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
boost::optional<gtsam::Pose3> omnimapper::OmniMapperBase::predictPose(
    gtsam::Symbol& pose_sym) {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (current_solution.exists<gtsam::Pose3>(pose_sym))
    return (current_solution.at<gtsam::Pose3>(pose_sym));
  else if (pose_plugins.size() > 0) {
    // This pose isn't in our SLAM problem yet, but may be pending addition
    for (std::list<omnimapper::PoseChainNode>::iterator itr = chain.end();
         itr != latest_committed_node; --itr) {
      if (itr->symbol == pose_sym) {
        // Estimate from latest commited time to this time, using first
        // available pose plugin
        // TODO: should a pose plugin be selectable through some other means?
        // if (pose_plugins.size () > 0)
        //{
        printf("Latest: %lu\n", latest_committed_node->symbol.index());
        if ((new_values.exists<gtsam::Pose3>(latest_committed_node->symbol)) &&
            !(current_solution.exists<gtsam::Pose3>(
                latest_committed_node->symbol))) {
          printf("THIS WASNT ACTUALLY COMMITTED PROPERLY!!!\n");
          exit(1);
        }

        gtsam::Pose3 prev_pose =
            current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr predicted_pose_factor =
            pose_plugins[0]->addRelativePose(latest_committed_node->time,
                                             latest_committed_node->symbol,
                                             itr->time, itr->symbol);
        gtsam::Pose3 incremental_prediction = predicted_pose_factor->measured();
        gtsam::Pose3 predicted_pose = prev_pose.compose(incremental_prediction);
        return (predicted_pose);
        //}
        // else
        //{
        // If we don't have any pose plugins, our best guess is the most
        // recently optimized pose.
        // printf ("RETURNING PREV POSE\n");
        // gtsam::Pose3 prev_pose = current_solution.at<gtsam::Pose3>
        // (latest_committed_node->symbol); return (prev_pose);
        //}
      }
    }
  }
  return (boost::none);
}

void omnimapper::OmniMapperBase::spin() {
  while (true) {
    spinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }
}

void omnimapper::OmniMapperBase::spinOnce() {
  // boost::mutex::scoped_lock (omnimapper_mutex);
  if (!initialized_) return;
  if (debug_) printf("OMB: spinning...\n");
  // If time to commit
  // std::list<omnimapper::PoseChainNode>::iterator next_node =
  // latest_committed_node; next_node++; if
  // (boost::posix_time::microsec_clock::local_time() - latest_commit_time >
  // boost::posix_time::seconds (commit_window))
  bool updated = commitNextPoseNode();

  if (new_factors.size() > 0) {
    optimize();
    updated = true;
  }

  if (updated) {
    // printf ("OMB: optimizing\n");
    // optimize ();
    updateOutputPlugins();
    // Print latest
    if (debug_) {
      gtsam::Pose3 new_pose_value =
          current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);
      printf("OMB Latest Pose: %lf %lf %lf\n", new_pose_value.x(),
             new_pose_value.y(), new_pose_value.z());
      printf("OMB Latest pose det: %lf\n",
             new_pose_value.rotation().matrix().determinant());
    }

  } else {
    if (debug_) printf("OMB: No new factors to optimize\n");
  }
}

gtsam::Pose3 omnimapper::OmniMapperBase::getLatestPose() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (latest_committed_node == chain.begin()) {
    return (gtsam::Pose3::identity());
  }
  gtsam::Pose3 new_pose_value =
      current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);
  return (new_pose_value);
}

void omnimapper::OmniMapperBase::getLatestPose(gtsam::Pose3& pose, Time& time) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (latest_committed_node == chain.begin()) {
    pose = gtsam::Pose3::identity();
    time = (*get_time_)();  // boost::posix_time::microsec_clock::local_time();
    return;
  }
  pose = current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);
  time = latest_committed_node->time;
  return;
}

void omnimapper::OmniMapperBase::updateOutputPlugins() {
  // boost::mutex::scoped_lock (omnimapper_mutex_);
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  boost::shared_ptr<gtsam::Values> vis_values(
      new gtsam::Values(current_solution));
  boost::shared_ptr<gtsam::NonlinearFactorGraph> vis_graph(
      new gtsam::NonlinearFactorGraph(current_graph));
  double start = pcl::getTime();
  for (int i = 0; i < output_plugins.size(); i++) {
    if (debug_)
      printf("Updating plugin %d with %lu values\n", i, vis_values->size());
    output_plugins[i]->update(vis_values, vis_graph);
  }
  double end = pcl::getTime();
  if (debug_)
    std::cout << "OmniMapperBase: updating output plugins took: "
              << double(end - start) << std::endl;
}

void omnimapper::OmniMapperBase::reset() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  // Clear state
  isam2 = gtsam::ISAM2();
  new_factors = gtsam::NonlinearFactorGraph();
  new_values = gtsam::Values();
  current_solution = gtsam::Values();
  current_graph = gtsam::NonlinearFactorGraph();
  chain.clear();
  time_lookup.clear();
  symbol_lookup.clear();

  latest_committed_node = chain.begin();
  latest_commit_time = (*get_time_)();

  initialized_ = false;
  initial_pose_ = gtsam::Pose3::identity();
  largest_pose_index = 0;
}
