#include <omnimapper/omnimapper_base.h>
#include <pcl/common/time.h>

omnimapper::OmniMapperBase::OmniMapperBase()
    : suppress_commit_window_(false),
      initial_pose_(gtsam::Pose3::identity()),
      largest_pose_index_(0),
      get_time_(new GetSystemTimeFunctor()),
      debug_(false),
      initialized_(false) {
  latest_committed_node_ = chain_.begin();
  latest_commit_time_ = (*get_time_)();
  commit_window_ = 3.0;
}

void omnimapper::OmniMapperBase::Reset() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  isam2_ = gtsam::ISAM2();
  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_ = gtsam::Values();
  current_solution_ = gtsam::Values();
  current_graph_ = gtsam::NonlinearFactorGraph();
  chain_.clear();
  time_lookup_.clear();
  symbol_lookup_.clear();
  latest_committed_node_ = chain_.begin();
  latest_commit_time_ = (*get_time_)();
  initialized_ = false;
  initial_pose_ = gtsam::Pose3::identity();
  largest_pose_index_ = 0;
}

void omnimapper::OmniMapperBase::SetInitialPose(
    const gtsam::Pose3& initial_pose) {
  initial_pose_ = initial_pose;
}

void omnimapper::OmniMapperBase::SetTimeFunctor(
    omnimapper::GetTimeFunctorPtr time_functor) {
  get_time_ = time_functor;
}

gtsam::NonlinearFactorGraph omnimapper::OmniMapperBase::GetGraph() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  return (current_graph_);
}

gtsam::Pose3 omnimapper::OmniMapperBase::GetLatestPose() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (latest_committed_node_ == chain_.begin()) {
    return (gtsam::Pose3::identity());
  }
  gtsam::Pose3 new_pose_value =
      current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol);
  return (new_pose_value);
}

void omnimapper::OmniMapperBase::GetLatestPose(gtsam::Pose3* pose, Time* time) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (latest_committed_node_ == chain_.begin()) {
    *pose = gtsam::Pose3::identity();
    *time = (*get_time_)();
    return;
  }
  *pose = current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol);
  *time = latest_committed_node_->time;
  return;
}

void omnimapper::OmniMapperBase::InitializePose(const Time& t) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  if (initialized_) {
    LOG(FATAL) << "OmniMapperBase is already initialized.";
    return;
  }

  // Adds the initial pose symbol.
  // TODO: Make the init symbol a default parameter, but changeable.
  gtsam::Symbol initial_symbol('x', 0);
  current_pose_symbol_ = initial_symbol;
  new_values_.insert(initial_symbol, initial_pose_);

  // Adds the prior pose factor.
  gtsam::Vector pose_prior_noise(6);
  pose_prior_noise << 0.001, 0.001, 0.001, 0.001, 0.001, 0.001;
  gtsam::PriorFactor<gtsam::Pose3> pose_prior(
      initial_symbol, initial_pose_,
      gtsam::noiseModel::Diagonal::Sigmas(pose_prior_noise);
  new_factors_.add(pose_prior);

  // Add the initial symbol to the pose chain.
  PoseChainNode init_node(t, initial_symbol);
  init_node.status = omnimapper::PoseChainNode::COMMITTED;
  std::list<omnimapper::PoseChainNode>::iterator new_itr =
      chain_.insert(chain_.end(), init_node);

  symbol_lookup_.insert({initial_symbol, new_itr});
  time_lookup_.insert({t, new_itr});

  Optimize();
  initialized_ = true;
  latest_committed_node_ = new_itr;

  LOG(INFO) << "Initialized OmnimapperBase pose.";
}

bool omnimapper::OmniMapperBase::AddFactorDirect(
    gtsam::NonlinearFactor::shared_ptr& new_factor) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  new_factors_.push_back(new_factor);
  return true;
}

void omnimapper::OmniMapperBase::UpdateOutputPlugins() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  UpdateOutputPluginsInternal();
}

void omnimapper::OmniMapperBase::UpdateOutputPluginsInternal() {
  boost::shared_ptr<gtsam::Values> vis_values(
      new gtsam::Values(current_solution_));
  boost::shared_ptr<gtsam::NonlinearFactorGraph> vis_graph(
      new gtsam::NonlinearFactorGraph(current_graph_));
  double start = pcl::getTime();

  for (std::size_t i = 0; i < output_plugins_.size(); i++) {
    LOG_IF(INFO, debug_) << "Updating plugin " << i
                         << " with " << vis_values->size() << "values.";
    output_plugins_[i]->Update(vis_values, vis_graph);
  }
  double end = pcl::getTime();
  LOG_IF(INFO, debug_) << "Updating output plugins took "
                       << double(end - start) << " seconds.";
}

void omnimapper::OmniMapperBase::Spin() {
  while (true) {
    SpinOnce();
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  }
}

void omnimapper::OmniMapperBase::SpinOnce() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  if (!initialized_) return;
  LOG_IF(INFO, debug_) << "OmniMapper is spinning...";

  const bool updated = CommitNextPoseNodeInternal();

  if (updated) {
    UpdateOutputPlugins();
    if (debug_) {
      gtsam::Pose3 new_pose_value =
          current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol);
      LOG(INFO) << "OmniMapper latest pose = (" << new_pose_value.x() << ", "
                << new_pose_value.y() << ", " << new_pose_value.z() << ")"
                << " det = "
                << new_pose_value.rotation().matrix().determinant();
    }
  } else {
    LOG_IF(INFO, debug_) << "OmniMapper has no new factors to optimize.";
  }
}

gtsam::NonlinearFactorGraph
omnimapper::OmniMapperBase::GetGraphAndUncommitted() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  gtsam::NonlinearFactorGraph graph = current_graph_;
  graph.push_back(new_factors_.begin(), new_factors_.end());
  return (graph);
}

gtsam::Values omnimapper::OmniMapperBase::GetSolution() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  return (current_solution_);
}

gtsam::Values omnimapper::OmniMapperBase::GetSolutionAndUncommitted() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  // Start with the  initial solution
  gtsam::Values solution = current_solution_;
  // Now add all landmarks from the new values
  solution.insert(new_values_);
  return (solution);
}

void omnimapper::OmniMapperBase::PrintSolution() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  current_solution_.print("Current OmniMapper Solution: \n");
  current_graph_.print("Current OmniMapper Graph: \n");
}

boost::optional<gtsam::Pose3> omnimapper::OmniMapperBase::GetPose(
    gtsam::Symbol& pose_sym) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  if (current_solution_.exists<gtsam::Pose3>(pose_sym))
    return (current_solution_.at<gtsam::Pose3>(pose_sym));
  else
    return (boost::none);
}

void omnimapper::OmniMapperBase::AddPosePlugin(
    const omnimapper::OmniMapperBase::PosePluginPtr& plugin) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  pose_plugins_.push_back(plugin);
}

void omnimapper::OmniMapperBase::AddOutputPlugin(
    const omnimapper::OmniMapperBase::OutputPluginPtr& plugin) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  output_plugins_.push_back(plugin);
}

bool omnimapper::OmniMapperBase::AddNewValue(const gtsam::Symbol& new_symbol,
                                             const gtsam::Value& new_value) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  LOG(INFO) << "Adding new symbol: " << new_symbol;
  new_values_.insert(new_symbol, new_value);
  return (true);
}

void omnimapper::OmniMapperBase::UpdateValue(const gtsam::Symbol& update_symbol,
                                             const gtsam::Value& update_value) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  LOG(INFO) << "Updating symbol: " << update_symbol;
  // Check new values first.
  if (new_values_.exists(update_symbol)) {
    new_values_.update(update_symbol, update_value);
    return;
  }

  // TODO: We patched ISAM2 to support following, should we enable it?
  LOG(FATAL) << "Update value not supported! COGROB: CHECK CODE IF WE NEED IT.";
  // gtsam::Values& state = isam2.getLinearizationPointUnsafe();
  // state.update(update_symbol, update_value);
}

void omnimapper::OmniMapperBase::UpdatePlane(const gtsam::Symbol& update_symbol,
                                             const gtsam::Pose3& pose,
                                             const gtsam::Plane<PointT>& meas_plane) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  LOG(INFO) << "Updating Plane symbol: " << update_symbol;
  if (new_values_.exists(update_symbol)) {
    gtsam::Plane<PointT> to_update =
        new_values_.at<gtsam::Plane<PointT> >(update_symbol);
    to_update.Extend2(pose, meas_plane);
    new_values_.update(update_symbol, to_update);
    return;
  }

  // TODO: We patched ISAM2 to support following, should we enable it?
  LOG(FATAL) << "Update value not supported! COGROB: CHECK CODE IF WE NEED IT.";
  // gtsam::Values& state = isam2.getLinearizationPointUnsafe();
  // gtsam::Plane<PointT> to_update =
  //     state.at<gtsam::Plane<PointT>>(update_symbol);
  // to_update.Extend2(pose, meas_plane);
  // to_update.Extend(pose, meas_plane);
  // state.update(update_symbol, to_update);
}

void omnimapper::OmniMapperBase::UpdateBoundedPlane(
    const gtsam::Symbol& update_symbol, const gtsam::Pose3& pose,
    const omnimapper::BoundedPlane3<PointT>& meas_plane) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  // TODO: We should not have factor specific update functions, they should be
  // derived from updateable value.
  LOG(INFO) << "Updating BoundedPlane symbol: " << update_symbol;
  if (new_values_.exists(update_symbol)) {
    const omnimapper::BoundedPlane3<PointT> to_update =
        new_values_.at<omnimapper::BoundedPlane3<PointT> >(update_symbol);
    to_update.extendBoundary(pose, meas_plane);
    // extendBoundary manipulates a cloud it has pointer to. The actual
    // manipulated data is not stored inside BoundedPlane3. So we don't need to
    // write to_update back.
  } else {
    const gtsam::Values& isam_values = isam2_.getLinearizationPoint();
    const omnimapper::BoundedPlane3<PointT>& to_update =
        isam_values.at<omnimapper::BoundedPlane3<PointT> >(update_symbol);
    to_update.extendBoundary(pose, meas_plane);
    // extendBoundary manipulates a cloud it has pointer to. The actual
    // manipulated data is not stored inside BoundedPlane3. So we don't need to
    // write to_update back.
  }
  return;
}

bool omnimapper::OmniMapperBase::AddFactor(
    const gtsam::NonlinearFactor::shared_ptr& new_factor) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  // Find the pose keys related to this factor, adding this to the latest one.
  const std::vector<gtsam::Key> keys = new_factor->keys();

  if (keys.size() > 0 && gtsam::symbolChr(keys[0]) == 'o') {
    LOG_IF(INFO, debug_) << "Adding object between factor.";
    new_factors_.push_back(new_factor);
    return true;
  }

  int latest_pose_idx = -1;
  // Probably nobody has data earlier than this?
  Time latest_pose_time = boost::posix_time::neg_infin;

  LOG_IF(INFO, debug_) << "AddFactor: starting to look at keys.";
  for (std::size_t i = 0; i < keys.size(); i++) {
    if (gtsam::symbolChr(keys[i]) == 'x') {
      LOG_IF(INFO, debug_) << "Going to compare keys.";
      CHECK(symbol_lookup_.exists(keys[i]));
      if (symbol_lookup_[keys[i]]->time > latest_pose_time) {
        latest_pose_idx = i;
        latest_pose_time = symbol_lookup_[keys[i]]->time;
      }
    }
  }
  // Add this factor to the pose chain at the latest pose.
  // TODO: handle factors unrelated to any pose.  We can probably just go ahead
  // and add them directly.
  if (latest_pose_idx == -1) {
    LOG(ERROR) << "No pose factor associated with this factor. Not supported!";
    return false;
  }

  LOG_IF(INFO, debug_) << "Adding factor to pose: "
                       << symbol_lookup_[keys[latest_pose_idx]]->symbol;
  // If that pose has been committed already, we can add this directly for the
  // next optimization run.
  if (symbol_lookup_[keys[latest_pose_idx]]->status ==
      omnimapper::PoseChainNode::COMMITTED) {
    new_factors_.push_back(new_factor);
    LOG_IF(INFO, debug_) << "New factor pushed back direclty.";
  } else {
    // If it hasn't yet been commited, we should add this to the pending
    // factors, causing pose factors to add constraints for this timestamp.
    symbol_lookup_[keys[latest_pose_idx]]->factors.push_back(new_factor);
    LOG_IF(INFO, debug_) << "Added to the pending factors.";
  }
  return true;
}

bool omnimapper::OmniMapperBase::CommitNextPoseNode() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  return CommitNextPoseNodeInternal();
}


bool omnimapper::OmniMapperBase::CommitNextPoseNodeInternal() {
  if (debug_ && false) {
    LOG(INFO) << "PoseChain size: " << chain_.size();
    for (const omnimapper::PoseChainNode& node: chain_) {
      LOG(INFO) << "Node " << node.symbol.chr() << " " << node.symbol.index()
                << ", time" << boost::posix_time::to_simple_string(node.time)
                << ", has " << node.factors.size() << " factors".
    }
  }

  // Do nothing if there are no new factors.
  std::list<omnimapper::PoseChainNode>::iterator to_commit =
      latest_committed_node_;
  to_commit++;
  LOG_IF(INFO, debug_) << "Latest node: "
      << latest_committed_node_->symbol.chr() << " "
      << latest_committed_node_->symbol.index();
  LOG_IF(INFO, debug_) << "To commit node: "
      << to_commit->symbol.chr() << " " << to_commit->symbol.index();

  if (to_commit == chain_.end()) {
    LOG(ERROR) << "Called CommitNextPoseNode "
               << "but latest node has already been committed."
    return false;
  }

  // Check that enough time has elapsed.
  if (!suppress_commit_window_) {
    if (!(((*get_time_)() - to_commit->time) >
          boost::posix_time::seconds(commit_window_))) {
      LOG_EVERY_N(INFO, 5) << "CommitNextPoseNode suppressed. "
                           << "Not time to commit yet."
      return false;
    }
  }

  LOG(INFO) << "OmniMapper start to try to commit a PoseChainNode.";

  // TODO: verify that this won't break our chain by inspecting pose information
  // Call pose plugins to add pose factors between previous chain time and
  // current pose time
  bool initialized = false;

  for (std::size_t i = 0; i < pose_plugins_.size(); i++) {
    // TODO: make this a boost::optional, in case the plugin is disabled or
    // unable to give a pose.
    // TODO: check new_pose_factor isn't boost::none.
    gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr new_pose_factor =
        pose_plugins_[i]->AddRelativePose(latest_committed_node_->time,
                                          latest_committed_node_->symbol,
                                          to_commit->time, to_commit->symbol);
    CHECK(new_pose_factor);
    // Initialize the value if we're the first one
    if (!initialized) {
      gtsam::Pose3 relative_pose = new_pose_factor->measured();
      // Compose the relative pose with the previously optimized pose
      gtsam::Pose3 new_pose_value =
          current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol)
              .compose(relative_pose);

      // Note: this is a workaround for a GTSAM bug, where compose (also
      // between) return invalid/nan poses for small motions.
      if (!(std::isfinite(new_pose_value.x()) &&
            std::isfinite(new_pose_value.y()) &&
            std::isfinite(new_pose_value.z()) &&
            std::isfinite(new_pose_value.rotation().matrix().determinant()))) {
        new_pose_value =
            current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol);
      }

      new_values_.insert(to_commit->symbol, new_pose_value);
      initialized = true;
    }

    new_factors_.push_back(new_pose_factor);
  }

  // Add the rest of the factors.
  if (!initialized) {
    // If we have no pose factors, we need a relative pose measurement to
    // initialize the pose.
    for (std::size_t i = 0; i < to_commit->factors.size(); i++) {
      gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr between =
          boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
              (to_commit->factors[i]));
      if (between) {
        gtsam::Pose3 relative_pose = between->measured();
        gtsam::Pose3 new_pose_value =
            current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol)
                .compose(relative_pose);

        // Note: this is a workaround for a GTSAM bug, where compose (also
        // between) return invalid/nan poses for small motions.
        if (!(std::isfinite(new_pose_value.x()) &&
              std::isfinite(new_pose_value.y()) &&
              std::isfinite(new_pose_value.z()) &&
              std::isfinite(
                  new_pose_value.rotation().matrix().determinant()))) {
          new_pose_value = current_solution_.at<gtsam::Pose3>(
              latest_committed_node_->symbol);
        }

        new_values_.insert(to_commit->symbol, new_pose_value);
        initialized = true;
        break;
      }
    }

    // If we didn't find any way to initialize the value, we can't commit yet!
    if (!initialized) {
      LOG_IF(INFO, debug_)
          << "OmniMapper: Tried to commit without any between factors! "
          << "Waiting for between factor. "
          << "Node has " << to_commit->factors.size() << " factors";
      }
      return false;
    }
  }

  LOG(INFO) << "OmniMapper will commit a PoseChainNode.";
  new_factors_.push_back(to_commit->factors);
  to_commit->status = omnimapper::PoseChainNode::COMMITTED;
  to_commit->factors.clear();
  latest_committed_node_++;
  latest_commit_time_ = (*get_time_)();

  OptimizeInternal();
  return true;
}

void omnimapper::OmniMapperBase::GetPoseSymbolAtTime(const Time& t,
                                                     gtsam::Symbol* sym) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  // If we haven't initialized yet, we do so on the first symbol request
  if (!initialized_) {
    LOG(INFO) << "Got symbol prior to initializing.";
    InitializePose(t);
  }

  // If we have a pose symbol for this timestamp, just return it.
  if (time_lookup_.count(t) > 0) {
    *sym = time_lookup_[t]->symbol;
    LOG_IF(INFO, debug_) << "We have this time already, returning it.";
    return;
  } else {
    // If we don't have a pose yet, make one
    LOG_IF(INFO, debug_) << "Make a new symbol for time " << t;
    // Check that the time is after the latest committed pose time, otherwise
    // we'd need to splice it into the pose chain, which is not yet supported.
    CHECK_GE(t, latest_committed_node_->time)
        << "OmniMapperBase: ERROR: Pose symbol requested for timestamp earlier "
           "than latest committed stamp! Increase the commit window length, or "
           "implement pose chain splicing (not yet implemented). "
        << "Requested: " << t ", Latest: " << latest_committed_node_->time;

    largest_pose_index_++;
    gtsam::Symbol new_sym('x', largest_pose_index_);
    *sym = new_sym;

    omnimapper::PoseChainNode new_node(t, new_sym);
    LOG_IF(INFO, debug_) << "OmniMapperBase: need new symbol at: " << t;

    // Find the node with the previous stamp.
    for (auto itr = chain_.end(), itr--; itr != chain_.begin(); itr--) {
      if (itr->time < t) break;
    }
    itr++;

    std::list<omnimapper::PoseChainNode>::iterator new_itr =
        chain_.insert(itr, new_node);
    time_lookup_.insert(std::make_pair(t, new_itr));
    symbol_lookup_.insert(std::make_pair(new_sym, new_itr));
    return;
  }
}

void omnimapper::OmniMapperBase::GetTimeAtPoseSymbol(const gtsam::Symbol& sym,
                                                     Time* t) {
  boost::lock_guard<boost::mutex> lock (omnimapper_mutex_);
  *t = symbol_lookup_[sym]->time;
}

void omnimapper::OmniMapperBase::Optimize() {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);
  OptimizeInternal();
}

void omnimapper::OmniMapperBase::OptimizeInternal() {
  double opt_start = pcl::getTime();
  LOG(INFO) << "Start to optimize.";

  LOG_IF(INFO, debug_) << "Current Solution: " << current_solution_;
  LOG_IF(INFO, debug_) << "Optimizing with new Factors: " << new_factors_;
  LOG_IF(INFO, debug_) << "Optimizing with new Values: " << new_values_;

  gtsam::ISAM2Result result = isam2_.update(new_factors_, new_values_);
  current_solution_ = isam2_.calculateEstimate();
  // TODO: is this necessary and okay?
  current_graph_ = isam2_.getFactorsUnsafe();
  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_.clear();
  double opt_end = pcl::getTime();
  LOG_IF(INFO, debug_) << "OmniMapper Optimize took "
                       << double(opt_end - opt_start) << " seconds.";
}

boost::optional<gtsam::Pose3> omnimapper::OmniMapperBase::PredictPose(
    gtsam::Symbol& pose_sym) {
  boost::lock_guard<boost::mutex> lock(omnimapper_mutex_);

  if (current_solution_.exists<gtsam::Pose3>(pose_sym)) {
    return (current_solution_.at<gtsam::Pose3>(pose_sym));
  } else if (pose_plugins_.size() > 0) {
    // This pose isn't in our SLAM problem yet, but may be pending addition.
    for (auto itr = chain_.end(), itr--; itr != latest_committed_node_; --itr) {
      if (itr->symbol == pose_sym) {
        // Estimate from latest commited time to this time, using first
        // available pose plugin.
        // TODO: should a pose plugin be selectable through some other means?

        LOG(INFO) << "Predict pose of " << pose_sym
                  << " from latest commited node "
                  << latest_committed_node_->symbol;
        if ((new_values_.exists<gtsam::Pose3>(
                latest_committed_node_->symbol)) &&
            !(current_solution_.exists<gtsam::Pose3>(
                latest_committed_node_->symbol))) {
          LOG(FATAL) << latest_committed_node_->symbol
                     << " was not actually committed properly.";
        }

        CHECK_GT(pose_plugins.size, 0) <<
            "Need at least one pose plugin to predict non-committed pose.";
        // TODO: If we don't have any pose plugins, our best guess is the most
        // recently optimized pose, that is
        // current_solution.at<gtsam::Pose3>(latest_committed_node->symbol);

        gtsam::Pose3 prev_pose =
            current_solution_.at<gtsam::Pose3>(latest_committed_node_->symbol);
        gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr predicted_pose_factor =
            pose_plugins_[0]->AddRelativePose(latest_committed_node_->time,
                                              latest_committed_node_->symbol,
                                              itr->time, itr->symbol);
        gtsam::Pose3 incremental_prediction = predicted_pose_factor->measured();
        gtsam::Pose3 predicted_pose = prev_pose.compose(incremental_prediction);
        return predicted_pose;
      }
    }
  }
  return boost::none;
}
