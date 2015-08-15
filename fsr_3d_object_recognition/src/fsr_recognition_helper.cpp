#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <tbb/tbb.h>

#include <fsr_threedorlib/fsr_recognition_helper.h>

#define FSR_RECOGNITION_HELPER_DEBUG 1
#define FSR_RECOGNITION_HELPER_VERBOSE 1

namespace fsr_or
{

  template <typename PointT>
  void ModelCache<PointT>::record ()
  {
    typename tbb::concurrent_unordered_map <OMKey::Ptr, CloudPtr, om_hash>::iterator it;
    for (it = cache_.begin (); it != cache_.end (); ++it)
    {
      track_[it->first] = false;
    }
  }

  template <typename PointT>
  void ModelCache<PointT>::getModel (const OMKey::Ptr &key, CloudPtr &cloud)
  {
    typename tbb::concurrent_unordered_map <OMKey::Ptr, CloudPtr, om_hash>::iterator it = cache_.find(key);
    if (it != cache_.end ())
    {
      cloud = it->second;
    }
    else
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::io::loadPCDFile (key->objectFileName().c_str (), *temp);
      pcl::copyPointCloud (*temp, *cloud);
      cache_[key] = cloud;
    }
    track_[key] = true;
  }

  template <typename PointT>
  void ModelCache<PointT>::clean ()
  {
    boost::unordered_map<OMKey::Ptr, bool, om_hash>::iterator it;
    for (it = track_.begin (); it != track_.end (); ++it)
    {
      if (!(it->second))
      {
        cache_.unsafe_erase (it->first);
      }
    }
    track_.clear ();
  }

}

template class fsr_or::ModelCache<pcl::PointXYZ>;
template class fsr_or::ModelCache<pcl::PointXYZRGBA>;
template class fsr_or::ModelCache<pcl::PointXYZRGB>;
