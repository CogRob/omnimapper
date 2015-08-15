#ifndef FSR_RECOGNITION_HELPER
#define FSR_RECOGNITION_HELPER

#include <fsr_threedorlib/fsr_types.h>
#include <tbb/concurrent_unordered_map.h>
#include <boost/array.hpp>

namespace fsr_or
{

  typedef boost::shared_ptr<Eigen::Matrix4f> MatrixTPtr;
  typedef Eigen::Matrix4f MatrixT;

  template <typename PointT>
  struct RegSolEntry
  {
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    const OMKey::Ptr info;
    CloudPtr cloud;
    MatrixT T;

    RegSolEntry () {}
    RegSolEntry (const OMKey::Ptr &info,
                 CloudPtr cloud,
                 MatrixT T)
    : info (info),
      cloud (cloud),
      T (T)
    {}
  };

  template <typename PointT>
  struct ModelCache
  {
    typedef pcl::PointCloud<PointT> Cloud;
    typedef typename Cloud::Ptr CloudPtr;

    typedef boost::shared_ptr<ModelCache> Ptr;

    tbb::concurrent_unordered_map <OMKey::Ptr, CloudPtr, om_hash> cache_;
    boost::unordered_map<OMKey::Ptr, bool, om_hash> track_;

    ModelCache () {}
    void getModel (const OMKey::Ptr &key, CloudPtr &cloud);
    void record ();
    void clean ();

    Ptr makeShared () const { return Ptr (new ModelCache (*this)); }
  };

}

#endif // FSR_RECOGNITION_HELPER
