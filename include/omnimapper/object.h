#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace omnimapper
{
  // For now, this is just a collection that contains a point cloud pointer and an optional label
  template <typename PointT>
  class Object
  {
    typedef typename pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;    
    
    public:
      // list of observations
      std::vector<CloudPtr> clusters;
      
      // optional label
      std::string name;

      // flag for use as a landmark
      bool landmark;
  };
  
}
