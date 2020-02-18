/*
 * Software License Agreement (BSD License)
 *
 *  OmniMapper
 *  Copyright (c) 2012-, Georgia Tech Research Corporation,
 *  Atlanta, Georgia 30332-0415
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

// STLs
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/FactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <omnimapper/BoundedPlane3.h>
#include <omnimapper/output_plugin.h>
#include <omnimapper/plane.h>
#include <omnimapper/pose_chain.h>
#include <omnimapper/pose_plugin.h>
#include <omnimapper/time.h>

#include <list>
#include <map>
#include <vector>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/thread.hpp>

// typedef pcl::PointXYZRGBA PointT;

// template class gtsam::Plane<PointT>;

namespace omnimapper {
/** \brief OmniMapperBase is the base class for the OmniMapper system.  It
 * contains a GTSAM-based factor graph and optimizer, as well as a variety of
 * helper functions for various SLAM tasks.  In particular, it handles adding
 * poses to the graph, which might come from a variety of sources such as robot
 * odometry, frame-to-frame ICP, IMU data, or any/all of the above. The mapper
 * also keeps a list of MeasurementPlugins, which allow various types of sensor
 * measurements to be added to the mapper.
 *
 * \author Alex Trevor, John Rogers
 */
class OmniMapperBase {
 public:
  // Helpful typedefs
  typedef boost::shared_ptr<gtsam::NonlinearFactor> NonlinearFactorPtr;
  typedef boost::shared_ptr<omnimapper::PosePlugin> PosePluginPtr;
  typedef boost::shared_ptr<omnimapper::OutputPlugin> OutputPluginPtr;

 protected:
  // An ISAM2 instance
  gtsam::ISAM2 isam2;
  // New factors to be added next optimization
  gtsam::NonlinearFactorGraph new_factors;
  // The initialization point for the new nodes
  gtsam::Values new_values;
  // The most recent solution after optimization
  gtsam::Values current_solution;
  // The most recent graph
  gtsam::NonlinearFactorGraph current_graph;
  // The symbol corresponding to the most recently added pose
  gtsam::Symbol current_pose_symbol;
  // The length of time in seconds to wait prior to committing new poses
  double commit_window;
  // Timestamp of the previous commit
  Time latest_commit_time;
  // flag for suppressing commit window
  bool suppress_commit_window_;
  // The pose to be initialized at
  gtsam::Pose3 initial_pose_;

  // The pose chain itself
  std::list<omnimapper::PoseChainNode> chain;
  // A pointer to the latest_committed_node
  std::list<omnimapper::PoseChainNode>::iterator latest_committed_node;
  // Largest used pose index (note that this is not necessarily the latest pose
  // temporally)
  int largest_pose_index;
  // For fast lookups, we keep a map of timestamps to nodes
  std::map<Time, std::list<omnimapper::PoseChainNode>::iterator> time_lookup;
  // For fast lookups, we keep a map of symbols to nodes too
  std::map<gtsam::Symbol, std::list<omnimapper::PoseChainNode>::iterator>
      symbol_lookup;
  // A source of time
  GetTimeFunctorPtr get_time_;

  // Mutex to protect the state
  boost::mutex omnimapper_mutex_;

  // std::vector<omnimapper::MeasurementPlugin> measurement_plugins;
  // A list of pose plugins.  The first plugin in the list will add the pose to
  // the graph and specify the initialization point, while the rest will only
  // add factors.
  std::vector<PosePluginPtr> pose_plugins;
  // A list of output plugins, for visualization, map publication, etc.
  std::vector<OutputPluginPtr> output_plugins;
  // Should add pose boost function pointer
  // boost::function<bool()> shouldAddPoseFn;

  // Debug mode
  bool debug_;
  // Triggered mode
  bool triggered_;
  // Initialized
  bool initialized_;

 public:
  /** \brief An empty constructor for the mapping base */
  OmniMapperBase();

  /** \brief Commits a pose in the pose chain to the SLAM problem.  Returns true
   * if updated, false otherwise. */
  bool commitNextPoseNode();

  /** \brief Adds an initial pose x_0 to the mapper. TODO: user specificed
   * initial pose */
  void initializePose(Time& t);

  /** \brief Sets the initial pose, to be initialized at the timestamp of the
   * first recieved message.  Has no effect if called when already initialized.
   */
  void setInitialPose(gtsam::Pose3& init_pose);

  /** \brief Sets a time functor to use for getting the current time. */
  void setTimeFunctor(omnimapper::GetTimeFunctorPtr time_functor);

  /** \brief Given a timestamp, return a pose symbol.  If a pose symbol already
   * exists for the requested timestamp, this is returned, else a new symbol is
   * created. */
  void getPoseSymbolAtTime(Time& t, gtsam::Symbol& sym);

  /** \brief Given a symbol, return the timestamp.  This is primarily used for
   * doing error analysis after mapping. */
  void getTimeAtPoseSymbol(gtsam::Symbol& sym, Time& t);

  /** \brief Returns the most recent solution */
  gtsam::Values getSolution();

  /** \brief Returs the most recent graph */
  gtsam::NonlinearFactorGraph getGraph();

  /** \brief Returs the most recent graph augmented with any pending uncommitted
   * graph*/
  gtsam::NonlinearFactorGraph getGraphAndUncommitted();

  /** \brief Returns the most recent solution augmented with any pending
   * uncommitted values */
  gtsam::Values getSolutionAndUncommitted();

  /** \brief Returns the most recent optimized pose. */
  gtsam::Pose3 getLatestPose();

  /** \brief Returns the most recent optimized pose, and the timestamp. */
  void getLatestPose(gtsam::Pose3& pose, Time& time);

  /** \brief Optimizes the graph.  This will update the SLAM problem with the
   * newly added factors, and optimize. */
  void optimize();

  /** \brief The main mapper update cycle, including adding poses, adding
   * measurements, checking for loop closures. */
  void spinOnce();

  /** \brief Continuously update the mapper while it is running. Suitable for
   * use in its own thread. */
  void spin();

  /** \brief Adds a pose plugin that will add a pose constraint when requested.
   */
  void addPosePlugin(PosePluginPtr& plugin);

  /** \brief Adds an output plugin, which will be called each time the map is
   * updated. */
  void addOutputPlugin(OutputPluginPtr& plugin);

  /** \brief Notify all output plugins that the state has changed. */
  void updateOutputPlugins();

  /** \brief SLAM systems assume measurements to be independent, so not every
   * available measurement should be used. Typically, a certain amount of
   * movement should have occured since the previous measurement.  shouldAddPose
   * determines when to add a new set of measurements, via movement amount (from
   * odometery, etc) or time, depending on the application.
   */
  // virtual bool
  // shouldAddPose ();

  /** \brief A function pointer can be specified to determine when the system
   * should add a new pose. This function should return true iff a new pose
   * should be added.  Can be the ready() function of a plugin.
   */
  // void
  // setShouldAddPoseFn (boost::function<bool()> fn);

  /** \brief Returns the most recent pose symbol. */
  // gtsam::Symbol
  // currentPoseSymbol ()
  //{
  //  return current_pose_symbol;
  //}

  /** \brief Adds a factor to the factor graph. */
  bool addFactor(gtsam::NonlinearFactor::shared_ptr& new_factor);

  /** \brief Adds a factor to the factor graph bypassing the pose chain. */
  bool addFactorDirect(gtsam::NonlinearFactor::shared_ptr& new_factor);

  /** \brief Adds an initial value to the values. */
  bool addNewValue(gtsam::Symbol& new_symbol, gtsam::Value& new_value);

  /** \brief Updates an existing value.  TODO: Fix this. */
  void updateValue(gtsam::Symbol& new_symbol, gtsam::Value& new_value);

  /** \brief Update a plane TODO: REMOVE THIS -- just adding this as a test.
   */
  void updatePlane(gtsam::Symbol& update_symbol, gtsam::Pose3& pose,
                   gtsam::Plane<pcl::PointXYZRGBA>& meas_plane);

  /* \brief Update a bounded plane -- TODO: remove this, should make
  updateable value. */
  void updateBoundedPlane(
      gtsam::Symbol& update_symbol, gtsam::Pose3& pose,
      omnimapper::BoundedPlane3<pcl::PointXYZRGBA>& meas_plane);

  /** \brief Looks up a pose by symbol. */
  boost::optional<gtsam::Pose3> getPose(gtsam::Symbol& pose_sym);

  /** \brief Predicts a pose that has not yet been committed / optimized. */
  boost::optional<gtsam::Pose3> predictPose(gtsam::Symbol& pose_sym);

  /** \brief Prints latest solution. */
  void printSolution();

  /** \brief Set whether or not to output verbose debugging information. */
  void setDebug(bool debug) { debug_ = debug; }

  void setSuppressCommitWindow(bool suppress) {
    suppress_commit_window_ = suppress;
  }

  /** \brief Resets the mapper, clearing all existing state. */
  void reset();

  // void
  // lock();

  // void
  // unlock();
};

}  // namespace omnimapper
