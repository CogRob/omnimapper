#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace omnimapper {

/** \brief PoseChainNode represents an entry in the pose chain.  */
struct PoseChainNode {
 public:
  typedef boost::posix_time::ptime Time;
  enum Status { UNCOMMITTED, COMMITTED };

  // The timestamp of this pose
  Time time;

  // The pose symbol for the mapper
  gtsam::Symbol symbol;

  // The status of this node
  Status status;

  // The list of factors to commit for this entry (only used for created and
  // staged status)
  std::vector<gtsam::NonlinearFactor::shared_ptr> factors;

  /* \brief Empty Constructor */
  PoseChainNode();

  PoseChainNode(Time& t, gtsam::Symbol& sym)
      : time(t), symbol(sym), status(UNCOMMITTED) {}
};

// /* \brief PoseChain manages the creation of poses for the mapper and the
// plugins, as well as the addition of poses and attached factors to the mapper.
// */
// class PoseChain {
//   public:
//     /* Empty Constructor */
//     PoseChain ()
//     {
//       largest_pose_index = -1;
//     }
//
//     /* Returns the pose symbol for time t */
//     void
//       getPoseSymbolAtTime (Time& t, gtsam::Symbol& sym)
//       {
//         if (time_lookup.count (t) > 0)
//         {
//           // If we have a pose, just return it
//           sym = time_lookup[t]->symbol ();
//           return;
//         }
//         else
//         {
//           // If we don't have a pose yet, make one
//           largest_pose_index++;
//           gtsam::Symbol new_sym ('x', largest_pose_index);
//           sym = new_symbol;
//           // Add a relevant node to the chain
//           omnimapper::PoseChainNode new_node (t, new_sym);
//           for (std::list<omnimapper::PoseChainNode>::iterator itr = chain.end
//               (); itr != chain.begin (); --itr)
//           {
//             if (itr->time () < t)
//             {
//               ++itr;
//               std::list<omnimapper::PoseChainNode>::iterator new_itr =
//                   chain.insert (itr, new_node); symbol_times.insert(
//                       std::pair<Time,
//                       std::list<omnimapper::PoseChainNode>::iterator>(
//                           t, new_itr));
//               break;
//             }
//           }
//           return;
//         }
//       }
//
//     /* Adds a factor */
//     void
//       addFactor (gtsam::NonlinearFactor::shared_ptr& new_factor)
//       {
//         // Get the pose key
//         if (symbol_times.count (t) == 0)
//         {
//           printf ("PoseChain: Attempting to add factor at invalid pose
//               key!\n"); return;
//         }
//         printf ("NYI\n");
//       }
//
//     /* Adds a relative pose measurement */
//     void
//       addRelativePoseMeasurement
//       (gtsam::BetweenFactor<gtsam::Pose3>::shared_ptr& pose_factor)
//       {
//         // Get start and end times
//         gtsam::Symbol start_sym = pose_factor->front ();
//         gtsam::Symbol end_sym = pose_factor->end ();
//
//         // We require that the start time has been already added to the pose
//         chain
//
//       }
//
//   protected:
//     // The pose chain itself
//     std::list<omnimapper::PoseChainNode> chain;
//     // A pointer to the latest_committed_node
//     PoseChainNode* latest_committed_node;
//     // Largest used pose index (note that this is not necessarily the latest
//     // pose temporally) int largest_pose_index;
//     // For fast lookups, we keep a map of timestamps to nodes
//     std::map<Time, std::list<omnimapper::PoseChainNode>::iterator>
//         time_lookup;
// };

}  // namespace omnimapper
