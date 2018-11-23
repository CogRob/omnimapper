#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <omnimapper/omnimapper_base.h>

#include <boost/thread/locks.hpp>

namespace omnimapper
{
  /** \brief Contains methods pertaining to each object
   */
  template<typename PointT>
  class Object
  {
      typedef typename pcl::PointCloud<PointT> Cloud;
      typedef typename Cloud::Ptr CloudPtr;
      typedef typename Cloud::ConstPtr CloudConstPtr;
      typedef typename std::pair<gtsam::Pose3, CloudPtr> PoseCloud;
    public:

      /** \brief Constructor
       */
      Object (gtsam::Symbol O);

      /**  \brief
       */
      Object (const Object& object);

      gtsam::Symbol sym; ///> Object symbol as used in the graph
      std::string name; ///> Object name

      std::map<gtsam::Symbol, PoseCloud > clusters_;
      std::map<gtsam::Symbol, pcl::PointIndices> indices_;
      std::map<gtsam::Symbol, int> factor_flag;

      CloudPtr optimal_cloud_; ///> Optimized cloud estimated every few seconds
      CloudPtr unoptimal_cloud_; ///> Unoptimized cloud given by object discovery, gets updated every time a segment is added to this object

      Eigen::Vector4f centroid_;
      std::pair<Eigen::Vector4f, Eigen::Vector4f> bounding_box_;

      boost::mutex object_mutex_; ///> Mutex

      bool landmark; ///> flag for use as a landmark in the factor graph
      bool debug_;
      bool verbose_;

      /**  \brief
             */
      void addObservation (gtsam::Symbol sym, CloudPtr cluster, gtsam::Pose3 cloud_pose);

      /**  \brief
             */
      Cloud optimizedCloud ();

      /**  \brief
             */
      Cloud unOptimizedCloud();

      /**  \brief
             */
      CloudPtr updateOptimizedCloud();

      /**  \brief
             */
      CloudPtr updateUnoptimizedCloud(gtsam::Symbol sym_latest, gtsam::Pose3 cloud_pose);

      /**  \brief
             */
      void updateCentroid(CloudPtr cloud);

      /**  \brief
                   */
      void updateBoundingBox(CloudPtr cloud);


     void saveAsPCD (std::string location);


      Eigen::Vector4f centroid (){return centroid_;}
      std::pair<Eigen::Vector4f, Eigen::Vector4f> boundingBox (){return bounding_box_;}

  };

}
