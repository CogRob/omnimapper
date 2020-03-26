#pragma once
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
//#include <gtsam/linear/SharedDiagonal.h>
#include <gtsam/linear/NoiseModel.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

/*!
 *\brief Sees if the displacement between these two poses exceeds distance
 *
 * \param last_pose   The previous pose where a message from this sensor message
 *was used to augment the map \param curr_pose   The current pose of the robot
 * \param distance    The required displacement for the new message to be
 *considered independent enough to incorporate into the map \return true if this
 *displacement is sufficient, meaning that this message should be used.  If this
 *is false, then it shouldnt be used.  This method ignores rotation
 *
 */

bool CheckMoveFarEnough(const tf::StampedTransform& last_pose,
                        const tf::StampedTransform& curr_pose, double distance,
                        double rot_thresh);

/*!
 * \brief Makes a gtsam::Pose2 out of a TF stamped transform
 *
 *\param transform   The TF::StampedTransform where the robot is to get the pose
 *from
 *
 * \return the gtsam::Pose2
 */

gtsam::Pose3 GetPose(const tf::StampedTransform& transform);

/*!
 * \brief Gets the relative pose between two transformations, used to compute
 *the odometric pose between two map robot poses
 *
 *\param last_pose   The last (odometric)pose of the robot, where the prior
 *message was used \param curr_pose   The current (odometric)pose of the robot
 * \return the pose transforming last_pose into curr_pose, which is the
 *measurement for the Pose2Factor
 */

gtsam::Pose3 GetRelativePose(const tf::StampedTransform& last_pose,
                             const tf::StampedTransform& curr_pose);

/*!
 *\brief   Takes an angle and wraps it to the range -M_PI:M_PI
 *
 *\param   in  an angle
 *\return  the angle in the range -M_PI:M_PI
 */
double WrapToPi(double in);

/*!
 * \brief Converts a Vector3 from TF to a Point3 for gtsam
 * \param vec   A Vector 3 from TF
 * \return a Point3 for GTSAM
 */
gtsam::Point3 btVectorToPoint3(const tf::Vector3& vec);

/*!
 *\brief Makes a GTSAM Pose3 out of a TF transform
 *
 *\param transform    A TF transform
 *\return the Pose3 equivalent for GTSAM
 */
gtsam::Pose3 BtTransformToPose3(const tf::Transform& transform);

tf::Transform Pose3ToRosTransform(const gtsam::Pose3& ps);

gtsam::Pose3 RosTransformToPose3(const tf::Transform& t);

gtsam::Pose2 Pose3ToPose2(const gtsam::Pose3& p3);
gtsam::Pose3 Pose2ToPose3(const gtsam::Pose2& p2);

gtsam::noiseModel::Diagonal::shared_ptr GetOdoCovariance(
    const gtsam::Pose3& odo, double sroll, double spitch, double syaw,
    double sx, double sy, double sz);
gtsam::Matrix CovarFromDiagonalNoiseModel(
    const gtsam::SharedDiagonal& diagonal);
