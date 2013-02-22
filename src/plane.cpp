/**
 * @file    Plane.cpp
 * @brief   3D Plane
 * @author  Alex Trevor
 * @author  John Rogers
 */

#include <omnimapper/plane.h>
//#include <gtpointcloud/pointcloud_helpers.h>
//#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/io/io.h>
//#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
//#include <mapping/pointcloud_helpers.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/geometry/polygon_operations.h>
#include <pcl/geometry/impl/polygon_operations.hpp>

using namespace std;

namespace gtsam {
  
  /* ************************************************************************* */
  Plane::Plane(): 
    a_(0), b_(0),c_(0),d_(0)
  {
    printf("Warning: Using default constructor in plane.  This might indicate a bug\n");
    //assert(false);
    concave_ = false;
  }

  //Plane::Plane(const gtsam::Pose3& pose, const omnimapper_msgs::PlaneInfo& plane_info, const bool& concave)
  Plane::Plane(const gtsam::Pose3& pose, const Plane& plane_info, const bool& concave)  
  {
    Eigen::Affine3f pose2map = pose3ToTransform(pose);
    pcl::PointCloud<Point> meas_hull = plane_info.hull();
    pcl::transformPointCloud(meas_hull,hull_,pose2map);
    
    Eigen::Vector4f map_normal;
    //gtsam::Point3 normal_in(plane_info.model.values[0],plane_info.model.values[1],plane_info.model.values[2]);
    gtsam::Point3 normal_in(plane_info.a(),plane_info.b(),plane_info.c());
    gtsam::Point3 normal_out = pose.rotation().rotate(normal_in);

    gtsam::Pose3 pose_inv = pose.inverse();

    a_ = normal_out.x();
    b_ = normal_out.y();
    c_ = normal_out.z();
    //    d_ = 
    //  plane_info.model.values[0] * pose_inv.x() + 
    //  plane_info.model.values[1] * pose_inv.y() + 
    //  plane_info.model.values[2] * pose_inv.z() + 
    //  plane_info.model.values[3];
    d_ = 
      plane_info.a() * pose_inv.x() +
      plane_info.b() * pose_inv.y() +
      plane_info.c() * pose_inv.z() +
      plane_info.d();
    concave_ = concave;

  }

  Plane::Plane(double a, double b,
	       double c, double d,
	       const pcl::PointCloud<Point>& hull,
	       const pcl::PointCloud<Point>& inliers,
	       const Eigen::Vector4f& centroid,
	       const std_msgs::Header header)
    : a_(a), b_(b), c_(c), d_(d),
      hull_(hull),
      inliers_(inliers),
      centroid_(centroid),
      header_(header)
  {
    concave_ = false;
  }

  Plane::Plane(double a, double b,
	       double c, double d,
	       const pcl::PointCloud<Point>& hull,
	       const pcl::PointCloud<Point>& inliers,
	       const std_msgs::Header header)
    : a_(a), b_(b), c_(c), d_(d),
      hull_(hull),
      inliers_(inliers),
      header_(header)
  {
    concave_ = false;
  }

  Plane::Plane(double a, double b, 
	       double c, double d, 
	       const pcl::PointCloud<Point>& hull, 
	       const pcl::PointCloud<Point>& inliers, 
	       const bool& concave)
    : a_(a), b_(b), c_(c), d_(d), 
      hull_(hull), 
      inliers_(inliers), 
      concave_(concave)
  {
  }


  void Plane::print(const string& s) const {
    cout << s << "(" << a_ << ", " << b_ << ", "<< c_<< ", "<< d_<< ")" << endl;
  }

  

  /* ************************************************************************* */
  bool Plane::equals(const Plane& q, double tol) const {
    return (fabs(a_ - q.a()) < tol && 
	    fabs(b_ - q.b()) < tol &&
	    fabs(c_ - q.c()) < tol &&
	    fabs(d_ - q.d()) < tol);
  }
 
 
  Plane Plane::retract(const Vector& d) const{
    if (d.size() == 0) {
      return *this;
    }

    Eigen::Vector3f new_norm(a_+d(0),b_+d(1),c_+d(2));
    new_norm.normalize();

    pcl::ModelCoefficients map_model;
    map_model.values.push_back(new_norm[0]);
    map_model.values.push_back(new_norm[1]);
    map_model.values.push_back(new_norm[2]);
    map_model.values.push_back(d_+d(3));
    pcl::PointCloud<Point> map_hull_on_map;
    pcl::ProjectInliers<Point> proj1;
    proj1.setModelType(pcl::SACMODEL_PLANE);
    proj1.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(hull_));
    proj1.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
    proj1.filter(map_hull_on_map);


    Plane new_p(new_norm[0],
		new_norm[1],
		new_norm[2],
		d_+d(3),map_hull_on_map,inliers_,concave_);

    return new_p;
  }
    

  Vector Plane::localCoordinates(const Plane& p2) const{
    assert(false);
    return gtsam::zero(3);
  }


  Vector Plane::GetXo(const gtsam::Pose3& xr) const {
     
    Eigen::Vector4f pred;
    gtsam::Point3 normal_in(a_, b_, c_);
    gtsam::Point3 normal_out = xr.rotation().unrotate(normal_in);
    pred[0] = normal_out.x();
    pred[1] = normal_out.y();
    pred[2] = normal_out.z();
    pred[3] = a_*xr.x() + b_*xr.y() + c_*xr.z() + d_;
    
    //get a point in the robot frame
    //gtsam::Vector rpy = xr.rotation().rpy();
    //tf::Transform posemap = btTransform(tf::createQuaternionFromRPY(rpy[0], 
    //								    rpy[1],
    //								    rpy[2]),
    //					btVector3(xr.x(),xr.y(),xr.z()));

    //tf::Transform map2pose = posemap.inverse();
    //pcl::PointCloud<Point> hull_robot;
    //pcl_ros::transformPointCloud(hull_,hull_robot,map2pose);
    
    return gtsam::Vector_(4,
			  pred[0],
			  pred[1],
			  pred[2],
			  pred[3]);
  }

  gtsam::Matrix Plane::GetDh1(const gtsam::Pose3& xr) const{
    Matrix Dh1 = gtsam::zeros(4,6);
    Matrix H1;
    gtsam::Point3 normal_in(a_, b_, c_);
    gtsam::Point3 normal_out = xr.rotation().unrotate(normal_in,H1);
    Eigen::Vector3d n(a_,b_,c_);
    Eigen::Vector3d p = xr.rotation().transpose() * n;
    insertSub(Dh1, H1, 0,0);
    Dh1(3,0) = 0;
    Dh1(3,1) = 0;
    Dh1(3,2) = 0;
    Dh1(3,3) = p (0);//a_;
    Dh1(3,4) = p (1);//b_;
    Dh1(3,5) = p (2);//c_;
    
    return Dh1;
  }
  gtsam::Matrix Plane::GetDh2(const gtsam::Pose3& xr) const{
    Matrix Dh2 = gtsam::zeros(4,4);
    Matrix H2;
    gtsam::Point3 normal_in(a_, b_, c_);
    gtsam::Point3 normal_out = xr.rotation().unrotate(normal_in,boost::none,H2);
    insertSub(Dh2,H2, 0, 0);

    Dh2(0,3) = 0;
    Dh2(1,3) = 0;
    Dh2(2,3) = 0;

    Dh2(3,0) = xr.x();
    Dh2(3,1) = xr.y();
    Dh2(3,2) = xr.z();
    Dh2(3,3) = 1;
    
    return Dh2;
  }

  gtsam::Vector Plane::Geth(const Vector& xo, const Vector& measured) const{
    //printf("geth: xo: %lf %lf %lf %lf\n",xo(0),xo(1),xo(2),xo(3));
    //printf("geth: ms: %lf %lf %lf %lf\n",measured(0),measured(1),measured(2),measured(3));
    Vector h = gtsam::zero(4);
    h[0] = xo(0) - measured(0);
    h[1] = xo(1) - measured(1);
    h[2] = xo(2) - measured(2);
    h[3] = xo(3) - measured(3);
    return h;
  }
  gtsam::Vector Plane::GetXf()const {
    return gtsam::Vector_(4,a_,b_,c_,d_);
  }

  //  gtsam::Vector Plane::GetLinearState(const gtsam::Pose3& xr,
  //				      const omnimapper_msgs::PlaneInfo& measured,
  //				      boost::optional<Matrix&> dhbydxr,
  //				      boost::optional<Matrix&> dhbydxf) const{
  gtsam::Vector Plane::GetLinearState(const gtsam::Pose3& xr,
				      const Plane& measured,
				      boost::optional<Matrix&> dhbydxr,
				      boost::optional<Matrix&> dhbydxf) const{

    gtsam::Vector xo = GetXo(xr);
    gtsam::Vector normal = gtsam::Vector_(4,
					  measured.a(),
					  measured.b(),
					  measured.c(),
					  measured.d());

    gtsam::Vector h = Geth(xo,normal);
    if(dhbydxr){
      *dhbydxr = GetDh1(xr);
    }
    if(dhbydxf){
      *dhbydxf = GetDh2(xr);
    }
    return h;
  }

  void Plane::Retract(const Pose3& pose, const gtsam::Plane& plane){
    //take just this hull one hull and project it back onto the model
    pcl::ModelCoefficients map_model;
    map_model.values.push_back(a_);
    map_model.values.push_back(b_);
    map_model.values.push_back(c_);
    map_model.values.push_back(d_);

    pcl::PointCloud<Point> meas_hull_in_map;

    //tf::Transform posemap = Pose3ToTransform(pose);
    Eigen::Affine3f posemap = pose3ToTransform(pose);
    pcl::transformPointCloud(plane.hull_,meas_hull_in_map,posemap);
    meas_hull_in_map.header.frame_id = "/map";

    pcl::PointCloud<Point> meas_hull_on_map;
    pcl::ProjectInliers<Point> proj1;
    proj1.setModelType(pcl::SACMODEL_PLANE);
    proj1.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(meas_hull_in_map));
    proj1.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
    proj1.filter(meas_hull_on_map);

    hull_ = meas_hull_on_map;
  }

  void Plane::Extend(const Pose3& pose, const gtsam::Plane& plane){
    //Make a model coefficients from our map normal
    pcl::ModelCoefficients map_model;
    map_model.values.push_back(a_);
    map_model.values.push_back(b_);
    map_model.values.push_back(c_);
    map_model.values.push_back(d_);

    //reproject map hull, in case of updates
    pcl::PointCloud<Point> map_hull_on_map;
    pcl::ProjectInliers<Point> proj1;
    proj1.setModelType(pcl::SACMODEL_PLANE);
    proj1.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(hull_));
    proj1.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
    proj1.filter(map_hull_on_map);
    map_hull_on_map.header.frame_id = "/map";

    //put this in the map frame
    //gtsam::Vector rpy = pose.rotation().rpy();

    pcl::PointCloud<Point> meas_hull_in_map;
    //tf::Transform posemap = btTransform(tf::createQuaternionFromRPY(rpy[0], 
    //								    rpy[1],
    //								    rpy[2]),
    //					btVector3(pose.x(),pose.y(),pose.z()));

    meas_hull_in_map.header.frame_id = "/map";
    //tf::Transform posemap = Pose3ToTransform(pose);
    Eigen::Affine3f posemap = pose3ToTransform(pose);
    pcl::transformPointCloud(plane.hull_,meas_hull_in_map,posemap);

    pcl::PointCloud<Point> meas_hull_on_map;
    pcl::ProjectInliers<Point> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(meas_hull_in_map));
    proj.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
    proj.filter(meas_hull_on_map);
    meas_hull_on_map.header.frame_id = "/map";

    pcl::PointCloud<Point> merged_cloud;
    merged_cloud.header = plane.hull_.header;
    merged_cloud.header.frame_id = "/map";
    merged_cloud += map_hull_on_map;//hull_;
    merged_cloud += meas_hull_on_map;

    pcl::PointCloud<Point> merged_hull;

    //TEST
    Eigen::Vector4f vec_model (map_model.values[0], map_model.values[1], map_model.values[2], map_model.values[3]);
    pcl::PlanarPolygon<Point> map_region (map_hull_on_map.points, vec_model);
    pcl::PlanarPolygon<Point> meas_region (meas_hull_on_map.points, vec_model);
    pcl::PlanarPolygon<Point> fused_region;
    pcl::PointCloud<Point> xy1;
    pcl::PointCloud<Point> xy2;
    
    //pcl::PlanarPolygon<Point> approx_map;
    //pcl::PlanarPolygon<Point> approx_meas;
    //pcl::approximatePolygon (map_region, approx_map, 0.005, false);
    //pcl::approximatePolygon (meas_region, approx_meas, 0.005, false);

    bool fused_worked = pcl::fusePlanarPolygons (map_region, meas_region, fused_region, vec_model, xy1, xy2);

    if (!fused_worked)
    {
      printf ("ERROR FUSING INSIDE PLANE!\n\n\n\n\n\n\n");
    }
    
    //TEST

    /*
    if(concave_){
      //project and merge inliers
      pcl::PointCloud<Point> map_inliers_on_map;
      pcl::ProjectInliers<Point> proj2;
      proj2.setModelType(pcl::SACMODEL_PLANE);
      proj2.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(inliers_));
      proj2.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
      proj2.filter(map_inliers_on_map);
      
      pcl::PointCloud<Point> meas_inliers_in_map;
      pcl::transformPointCloud(plane.inliers_,meas_inliers_in_map,posemap);
      
      pcl::PointCloud<Point> meas_inliers_on_map;
      pcl::ProjectInliers<Point> proj3;
      proj3.setModelType(pcl::SACMODEL_PLANE);
      proj3.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(meas_inliers_in_map));
      proj3.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(map_model));
      proj3.filter(meas_inliers_on_map);

      pcl::PointCloud<Point> merged_cloud_full;
      merged_cloud_full.header = hull_.header;
      merged_cloud_full += map_inliers_on_map;
      merged_cloud_full += meas_inliers_on_map;

      inliers_ = merged_cloud_full;

      pcl::ConcaveHull<Point> chull;
      std::vector<pcl::Vertices> pgons;
      chull.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(merged_cloud_full));
      chull.setAlpha(0.15);
      chull.reconstruct(merged_hull,pgons);
    } else {
    */
    //pcl::ConvexHull<Point> chull;
    // chull.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(merged_cloud));
    //  chull.reconstruct(merged_hull);
      //}
    //hull_ = merged_hull;

    std::vector<int> hull_inds;
    getConvexHull2D(merged_cloud.points,hull_inds);
    pcl::ExtractIndices<Point> extract;
    pcl::PointIndices hull_pi;
    hull_pi.indices = hull_inds;
    extract.setInputCloud(boost::make_shared<pcl::PointCloud<Point> >(merged_cloud));
    extract.setIndices(boost::make_shared<pcl::PointIndices>(hull_pi));
    extract.filter(merged_hull);
    //hull_ = merged_hull;
    if (fused_worked)
      hull_.points = fused_region.getContour ();
    else
      hull_ = map_hull_on_map;

  }

  /* ************************************************************************* */
} // namespace gtsam
