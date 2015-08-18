#include <pcl/filters/filter.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_impl.h>
#include <tbb/tbb.h>

#include <fsr_threedorlib/fsr_hashdescription.h>

#define FSR_HASHDESCRIPTION_DEBUG 1
#define FSR_HASHDESCRIPTION_VERBOSE 1

namespace fsr_or
{

  template <typename PointT>
  FSRHashMapDescription<PointT>::FSRHashMapDescription (float d,
                                                        float delta_d,
                                                        float K,
                                                        float angle_step)
  : d_min_ (d - delta_d),
    d_max_ (d + delta_d),
    d_min_sq_ (pow (d - delta_d, 2)),
    d_max_sq_ (pow (d + delta_d, 2)),
    K_ (K),
    angle_step_ (angle_step),
    m_ (0),
    H_ (new FeatureHashMap<PointT>),
    omkey_box_ ()
  {
    H_->clear ();
    omkey_box_.clear ();
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::addModelToFile (CloudPtr &cloud,
                                                      std::string model_name, int model_type,
                                                      std::ofstream &mfile) const
  {
    /// TODO: add resolution invariance

    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: adding model " << model_name << " of type "
              << model_type << "." << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    std::vector<int> pt_indices;
    if (!validateCloudForProcessing (cloud, pt_indices))
    {
      #if FSR_HASHDESCRIPTION_DEBUG
      std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model did not have enough finite pts!" << std::endl;
      #endif // FSR_HASHDESCRIPTION_DEBUG
      return;
    }
    #if FSR_HASHDESCRIPTION_DEBUG
    else
    {
      std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model has " << pt_indices.size ()
                << " finite points."<< std::endl;
    }
    #endif // FSR_HASHDESCRIPTION_DEBUG

    CloudPtr cloud_input (new Cloud ());
    pcl::octree::OctreePointCloudVoxelCentroid<PointT> oct (0.01);
    oct.setInputCloud (cloud);
    oct.addPointsFromInputCloud ();
    typename pcl::octree::OctreePointCloud<PointT>::AlignedPointTVector centroids;
    size_t reducedsize = oct.getVoxelCentroids (centroids);
    if (reducedsize != cloud->width)
    {
      cloud_input->width = reducedsize;
      cloud_input->height = 1;
      cloud_input->points.resize (cloud_input->width * cloud_input->height);
      for (size_t i = 0; i < cloud_input->width; ++i)
      {
        cloud_input->points[i] = centroids[i];
      }
    }
    else
    {
      pcl::copyPointCloud (*cloud, pt_indices, *cloud_input);
    }
    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: the downsampled model has "
              << cloud_input->points.size () << " points." << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    NormalCloudPtr cloud_normals (new NormalCloud ());
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setRadiusSearch (0.03);
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_input);
    ne.setSearchSurface (cloud);
    ne.compute (*cloud_normals);

    std::vector<int> ne_indices;
    pcl::removeNaNNormalsFromPointCloud (*cloud_normals, *cloud_normals, ne_indices);

    static int model_count_all = 0;
    ++model_count_all;

    if (cloud_normals->points.size() < 2)
    {
      #if FSR_HASHDESCRIPTION_DEBUG
      std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model does not have more than 2 normals!" << std::endl;
      #endif // FSR_HASHDESCRIPTION_DEBUG
      return;
    }
    #if FSR_HASHDESCRIPTION_VERBOSE
    else
    {
      std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model has " << ne_indices.size()
                << " finite normals."<< std::endl;
    }
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    FeatureHashMapSingle mapformodel;
    FeatureTracker featurerecord;
    OrientedPointCloud cloud_opp (cloud_input, cloud_normals, ne_indices);
    #if FSR_HASHDESCRIPTION_DEBUG
    tbb::concurrent_vector<std::string> errs;
    #endif // FSR_HASHDESCRIPTION_DEBUG
    tbb::parallel_for (size_t(0), cloud_opp.points.size () - 1,
                      [&] (size_t i)
    {
      tbb::parallel_for (tbb::blocked_range<size_t> (i + 1, cloud_opp.points.size ()),
                         [&] (const tbb::blocked_range<size_t> &r)
      {
        for(size_t j = r.begin (); j != r.end (); ++j)
        {
          OrientedPoint<PointT> u (cloud_opp.points[i]);
          OrientedPoint<PointT> v (cloud_opp.points[j]);
          float f1, f2, f3, f4;
          if (computeOPFeature<PointT> (u, v,
                                        f1, f2, f3, f4,
                                        d_min_sq_, d_max_sq_))
          {
            int d2, d3, d4;
            Eigen::Matrix4f F;
            PointPair<PointT> ptpair (u, v);
            if (computeF<PointT> (ptpair, F))
            {
              discretizeOPFeatureVals (f2, f3, f4,
                                       d2, d3, d4,
                                       angle_step_);
              FSRFeature feature (d2, d3, d4);
              featurerecord.insert (feature);
              mapformodel.insert (std::pair<FSRFeature, PointPairSystem<PointT> > (feature, PointPairSystem<PointT> (ptpair, F)));
            }
            #if FSR_HASHDESCRIPTION_DEBUG
            else
            {
              std::stringstream ss;
              ss.clear ();
              ss.str (std::string ());
              ss << "[fsr_or::FSRHashMapDescription::addModelToFile()]: something went wrong trying to compute F for pair "
                 << ptpair.display () << "!" << "\n";
              errs.push_back(ss.str ());
            }
            #endif // FSR_HASHDESCRIPTION_DEBUG
          }
        }
      });
    });

    #if FSR_HASHDESCRIPTION_DEBUG
    for (size_t i = 0; i < errs.size (); ++i)
    {
      std::cout << errs[i] << std::endl;
    }
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: there were " << errs.size () << " errors" << std::endl;
    #endif // FSR_HASHDESCRIPTION_DEBUG

    int pretrim, postrim;
    trimFeaturesFromSingleTable (mapformodel, featurerecord, pretrim, postrim);

    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: pairs before trimming : "
              << pretrim << std::endl;
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: pairs after trimming : "
              << postrim << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    std::stringstream ss;
    ss.clear ();
    ss.str (std::string ());

    mfile << "{\n";
    ss << model_name << "," << model_type << "," << ne_indices.size () << "\n";
    mfile << ss.str ();
    writeModel (mapformodel, featurerecord, mfile);
    mfile << "}\n";

    static int model_count_succ = 0;
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: " << ++model_count_succ
              << " models out of " << model_count_all << " have been added to the hash map." << std::endl;
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::writeModel (const FeatureHashMapSingle &singlemap, const FeatureTracker &tracker, std::ofstream &mfile) const
  {
    FeatureTracker::const_iterator it;
    std::pair<FeatureHashMapSingleIterator, FeatureHashMapSingleIterator> fhmsitr;
    for (it = tracker.begin (); it != tracker.end (); ++it)
    {
      mfile << it->print () << ":\n";
      fhmsitr = singlemap.equal_range (*it);
      for (; fhmsitr.first != fhmsitr.second; ++fhmsitr.first)
      {
        mfile << fhmsitr.first->second.print ();
      }
    }
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::readModelsFromFile (std::string &fname)
  {
    std::ifstream mfile (fname);
    #if FSR_HASHDESCRIPTION_VERBOSE
    int c = 0;
    #endif // FSR_HASHDESCRIPTION_VERBOSE
    while (readModel(mfile))
    {
      ++c;
    }
    m_ /= c;
    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::readModelsFromFile()]: " << c << " models read " << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE
    mfile.close ();
  }

  template <typename PointT>
  bool FSRHashMapDescription<PointT>::readModel (std::istream &mfile)
  {
    std::string line;

    /// check for brace this way because line breaks are getting added between
    /// models in a random fashion for some reason
    do
    {
      std::getline (mfile, line);
      if (mfile.eof ())
      {
        return false;
      }
    }
    while (line.compare("{") != 0);

    std::getline (mfile, line); /// get model info
    std::stringstream ss (line);
    std::string value;

    /// read model name, type, and view
    std::getline (ss, value, ',');
    std::string mname = value;
    std::getline (ss, value, ',');
    int mtype = std::stoi (value);
    OMKey key (mname, mtype);
    omkey_box_.push_back(key.makeShared());

    /// get model number of points in model
    std::getline(ss, value, ',');
    int msize = std::stoi (value);
    m_ += msize;

    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::readModel()]: reading model " << mname << " of type "
              << mtype << "." << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    FSRFeature feature;
    std::getline (mfile, line);
    bool ppsfound = false;
    while (line.compare ("}") != 0)
    {
      /// read FSRFeature
      //std::cout << line << std::endl;
      FSRFeature::read(line, feature);

      /// read point pairs
      PointPairSystem<PointT> pps;
      std::vector<PointPairSystem<PointT> > systems;
      std::getline (mfile, line);
      while (PointPairSystem<PointT>::read (line, pps))
      {
        //std::cout << line << std::endl;
        systems.push_back (pps);
        std::getline (mfile, line);
        ppsfound = true;
      }
      //std::cout << line << std::endl;

      if (ppsfound)
      {
        H_->insert_model (feature, key, systems);
        ppsfound = false;
      }
    }

    return true;
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::trimFeaturesFromSingleTable (FeatureHashMapSingle &singlemap, const FeatureTracker &tracker, int &sum1, int &sum2) const
  {
    sum1 = static_cast<int> (singlemap.size ());

    int sum = 0, target = 0, upper = 0, lower = 0, c = 0, i = -1;
    FeatureTracker::const_iterator it;
    std::pair<FeatureHashMapSingleIterator, FeatureHashMapSingleIterator> fhmsitr;
    std::vector<int> fcounts (tracker.size (), 0);
    for (it = tracker.begin (); it != tracker.end (); ++it)
    {
      fhmsitr = singlemap.equal_range (*it);

      for (; fhmsitr.first != fhmsitr.second; ++fhmsitr.first) { ++c; }

      if (c > upper) { upper = c; }

      fcounts[++i] = c;
      c = 0;
    }

    sum = static_cast<int> (singlemap.size ());
    target = static_cast<int> (ceil (K_ * sum));
    lower = upper - 1;

    bool modified = false;
    while (sum > target)
    {
      i = -1;
      for (it = tracker.begin (); (it != tracker.end () && sum > target); ++it)
      {
        c = fcounts[++i];
        if (c >= lower)
        {
          singlemap.unsafe_erase (*it);
          modified = true;
          --sum;
        }
      }

      --upper;
      --lower;
      modified = false;
    }

    sum2 = sum;
  }

  template <typename PointT>
  bool FSRHashMapDescription<PointT>::validateCloudForProcessing (const CloudPtr &cloud, std::vector<int> &pt_indices) const
  {
    CloudPtr cloud_test (new Cloud ());
    pcl::removeNaNFromPointCloud (*cloud, *cloud_test, pt_indices);
    if (pt_indices.size() < 100)
    {
      #if FSR_HASHDESCRIPTION_VERBOSE
      std::cout << "[fsr_or::FSRHashMapDescription::validateCloudForProcessing()]: cloud only has " << pt_indices.size()
                << " finite pts!" << std::endl;
      #endif // FSR_HASHDESCRIPTION_VERBOSE
      return false;
    }
    return true;
  }

}

template class fsr_or::FSRHashMapDescription<pcl::PointXYZ>;
template class fsr_or::FSRHashMapDescription<pcl::PointXYZRGBA>;
template class fsr_or::FSRHashMapDescription<pcl::PointXYZRGB>;
