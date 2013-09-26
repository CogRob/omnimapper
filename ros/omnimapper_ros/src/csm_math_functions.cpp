#include <omnimapper_ros/csm_math_functions.h>

#if ROS_VERSION > ROS_VERSION_COMBINED(1,8,0)
#define btVector3 tf::Vector3
#define btMatrix3x3 tf::Matrix3x3
#endif

bool CheckMoveFarEnough(const tf::StampedTransform& last_pose, 
			const tf::StampedTransform& curr_pose, 
			double distance,
			double rot_thresh) {
  double dx = curr_pose.getOrigin().x() - last_pose.getOrigin().x();
  double dy = curr_pose.getOrigin().y() - last_pose.getOrigin().y();
  double dz = curr_pose.getOrigin().z() - last_pose.getOrigin().z();
  double dist = sqrt(dx*dx + dy*dy + dz*dz);
  double ang_dist = fabs(WrapToPi(tf::getYaw(last_pose.getRotation()) - tf::getYaw(curr_pose.getRotation())));
  return (dist >= distance) || (ang_dist >= rot_thresh);
}

gtsam::Pose3 GetPose(const tf::StampedTransform& transform) {
  //  return gtsam::Pose2(transform.getOrigin().x(),
  //		      transform.getOrigin().y(),
  //		      tf::getYaw(transform.getRotation()));
  btVector3 axis = transform.getRotation().getAxis();
  gtsam::Vector gtsam_axis;
  double axis_norm = sqrt(axis[0]*axis[0] +
			  axis[1]*axis[1] + 
			  axis[2]*axis[2]);
  if (fabs(axis_norm - 1.0) < 0.01) {
    gtsam_axis = gtsam::Vector_(3,axis[0]/axis_norm,axis[1]/axis_norm,axis[2]/axis_norm);
  } else {
    printf("Axis failure !  Axis length %lf\n", 
	   axis_norm);
    gtsam_axis = gtsam::Vector_(3,0,0,1);
  }
  double angle = transform.getRotation().getAngle();
  return gtsam::Pose3(gtsam::Rot3::rodriguez(gtsam_axis,angle),
		      gtsam::Point3(transform.getOrigin().x(),
				    transform.getOrigin().y(),
				    transform.getOrigin().z()));
}
gtsam::Pose3 GetRelativePose(const tf::StampedTransform& last_pose,
			     const tf::StampedTransform& curr_pose) {

  gtsam::Pose3 initial = GetPose(last_pose);
  gtsam::Pose3 final = GetPose(curr_pose);

  gtsam::Pose3 odo = initial.between(final);

  return odo;
}

tf::Transform Pose3ToTransform(const gtsam::Pose3& ps) {
  tf::Transform t;
  t.setOrigin(btVector3(ps.x(), ps.y(), ps.z()));
  tf::Quaternion q;
  gtsam::Vector ypr = ps.rotation().ypr();
  q.setRPY(ypr[2], ypr[1], ypr[0]);
  t.setRotation(q);
  return t;
}
gtsam::Pose3 TransformToPose3(const tf::Transform& t) {
  double roll, pitch, yaw;
  btMatrix3x3 m(t.getRotation());
  m.getRPY(roll, pitch, yaw);
  return gtsam::Pose3(gtsam::Rot3::ypr(yaw,
				       pitch,
				       roll),
		      gtsam::Point3(t.getOrigin().x(),
				    t.getOrigin().y(),
				    t.getOrigin().z()));

}

tf::Transform Pose2ToTransform(const gtsam::Pose2& ps) {
  tf::Transform t;
  t.setOrigin(btVector3(ps.x(), ps.y(), 0.0));
  tf::Quaternion q;
  q.setRPY(0,0,ps.theta());
  t.setRotation(q);
  return t;
}
gtsam::Pose2 TransformToPose2(const tf::Transform& t) {
  double roll, pitch, yaw;
  btMatrix3x3 m(t.getRotation());
  m.getRPY(roll, pitch, yaw);
  return gtsam::Pose2(yaw,
		      gtsam::Point2(t.getOrigin().x(),
				    t.getOrigin().y()));

}

gtsam::Point3 btVectorToPoint3(const btVector3& vec) {
  return gtsam::Point3(vec.getX(),vec.getY(),vec.getZ());
}
gtsam::Pose3 btTransformToPose3(const tf::Transform& transform) {
  btVector3 col0 = transform.getBasis().getColumn(0);
  btVector3 col1 = transform.getBasis().getColumn(1);
  btVector3 col2 = transform.getBasis().getColumn(2);
  gtsam::Rot3 rot(btVectorToPoint3(col0),
		  btVectorToPoint3(col1),
		  btVectorToPoint3(col2));
  gtsam::Point3 trans = btVectorToPoint3(transform.getOrigin());
  return gtsam::Pose3(rot, trans);
}

double WrapToPi(double diff){
  double ret = diff;
  while(ret > M_PI){
    ret -= 2.0*M_PI;
  }
  while(ret  < -M_PI){
    ret += 2.0*M_PI;
  }
  return ret;
}

gtsam::noiseModel::Diagonal::shared_ptr 
GetOdoCovariance( const gtsam::Pose3& odo,
		  double sroll, double spitch, double syaw,
		  double sx, double sy, double sz) {
  double l_scale = sqrt(odo.x()*odo.x() + 
			odo.y()*odo.y() + 
			odo.z()*odo.z())/1.5;
  if (l_scale < 0.1) l_scale = 0.1;
  gtsam::Vector ypr = odo.rotation().ypr();
  double a_scale = sqrt(ypr[0]*ypr[0] +
			ypr[1]*ypr[1] +
			ypr[2]*ypr[2])/1.5;
  if (a_scale < 0.1) a_scale = 0.1;
  
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector_(6,
					    a_scale*(sroll),
					    a_scale*(spitch),
					    a_scale*(syaw),
					    l_scale*(sx),
					    l_scale*(sy),
					    l_scale*(sz)));
}
gtsam::Matrix CovarFromDiagonalNoiseModel(const gtsam::SharedDiagonal& diagonal) {
  const gtsam::Vector sigmas = diagonal->sigmas();
  gtsam::Matrix result = gtsam::zeros(sigmas.size(), sigmas.size());
  for (unsigned int i = 0;i<sigmas.size();i++) {
    result(i,i) = sigmas(i) * sigmas(i);
  }
  return result;
}
gtsam::Pose2 Pose3ToPose2(const gtsam::Pose3& p3) {
  gtsam::Vector ypr = p3.rotation().ypr();
  return gtsam::Pose2(ypr[0],
		      gtsam::Point2(p3.x(),
				    p3.y()));

}
gtsam::Pose3 Pose2ToPose3(const gtsam::Pose2& p2) {
  return gtsam::Pose3(gtsam::Rot3::ypr(p2.theta(),
				       0,0),
		      gtsam::Point3(p2.x(),
				    p2.y(),
				    0.0));

}
