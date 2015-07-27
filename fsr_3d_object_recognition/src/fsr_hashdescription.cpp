#include <pcl/filters/filter.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <tbb/tbb.h>

#include <fsr_threedorlib/fsr_hashdescription.h>

#define FSR_HASHDESCRIPTION_DEBUG 1
#define FSR_HASHDESCRIPTION_VERBOSE 1

namespace fsr_or
{
  template <typename PointT>
  void FSRHashMapDescription<PointT>::addModelToFile (CloudConstPtr &cloud_input,
                                                 std::string &model_name, int &model_type, int &model_view,
                                                 std::ofstream &mfile)
  {
    /// TODO: add resolution invariance

    if (!validateCloudForProcessing (cloud_input))
    {
      #if FSR_HASHDESCRIPTION_DEBUG
      std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model did not have enough finite pts!" << std::endl;
      #endif // FSR_HASHDESCRIPTION_DEBUG
      return;
    }
    #if FSR_HASHDESCRIPTION_DEBUG
    else
    {
      std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model has " << cloud_input->points.size()
                << " finite points."<< std::endl;
    }
    #endif // FSR_HASHDESCRIPTION_DEBUG

    NormalCloudPtr cloud_normals (new NormalCloud ());
    if (cloud_input->height > 1)
    {
      pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
      ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
      ne.setMaxDepthChangeFactor (0.02f);
      ne.setNormalSmoothingSize (10.0f);
      ne.setInputCloud (cloud_input);
      ne.compute (*cloud_normals);
    }
    else
    {
      typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
      pcl::NormalEstimation<PointT, pcl::Normal> ne;
      ne.setRadiusSearch (0.03);
      ne.setSearchMethod (tree);
      ne.setInputCloud (cloud_input);
      ne.compute (*cloud_normals);
    }

    std::vector<int> ne_indices;
    pcl::removeNaNNormalsFromPointCloud (*cloud_normals, *cloud_normals, ne_indices);

    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: this model has " << cloud_normals->points.size()
              << " finite normals."<< std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    float maxdist = -1.0;
    float d = d_ + delta_d_;
    FeatureHashMapSingle mapformodel;
    #if FSR_HASHDESCRIPTION_DEBUG
    tbb::concurrent_vector<std::string> errs;
    #endif // FSR_HASHDESCRIPTION_DEBUG
    //for (size_t i = 0; i < ne_indices.size (); i++)
    tbb::parallel_for(0, static_cast<int>(ne_indices.size ()), 1, [&] (int i)
    {
      //for (size_t j = i + 1; j < ne_indices.size (); j++)
      tbb::parallel_for(i + 1, static_cast<int>(ne_indices.size ()), 1, [&] (int j)
      {
        float f1, f2, f3, f4;
        PointPair<PointT> ptpair;
        ptpair.first.p = cloud_input->points[ne_indices[i]];
        ptpair.first.n = cloud_normals->points[i];
        ptpair.second.p = cloud_input->points[ne_indices[j]];
        ptpair.second.n = cloud_normals->points[j];
        if (computeOPFFeature (ptpair.first.p.getVector4fMap (),
                               ptpair.first.n.getNormalVector4fMap (),
                               ptpair.second.p.getVector4fMap (),
                               ptpair.second.n.getNormalVector4fMap (),
                               f1, f2, f3, f4,
                               d))
        {
          int d2, d3, d4;
          Eigen::Matrix4f F;
          d2 = static_cast<int> (floor (f2 / angle_step_));
          d3 = static_cast<int> (floor (f3 / angle_step_));
          d4 = static_cast<int> (floor (f4 / angle_step_));
          if (computeF(ptpair, F))
          {
            typename FeatureHashMapSingle::accessor a;
            FSRFeature feature (d2, d3, d4);
            mapformodel.insert (a, feature);
            a->second.push_back(PointPairSystem<PointT> (ptpair, F));
            a.release ();
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

          {
            DiamMutex::scoped_lock lock (dmutex);
            if (f1 > maxdist)
            {
              maxdist = f1;
            }
          }
        }
      });
    });

    #if FSR_HASHDESCRIPTION_DEBUG
    for (size_t i = 0; i < errs.size (); i++)
    {
      std::cout << errs[i] << std::endl;
    }
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: there were " << errs.size () << std::endl;
    #endif // FSR_HASHDESCRIPTION_DEBUG

    trimFeaturesFromSingleTable (mapformodel);

    std::stringstream ss;
    ss.clear ();
    ss.str (std::string ());

    mfile << "{\n";
    ss << model_name << "," << model_type << "," << model_view << "," << maxdist << "," << ne_indices.size () << "\n";
    mfile << ss.str ();
    writeModel (mapformodel, mfile);
    mfile << "}\n";

    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::addModelToFile()]: added model " << model_name << " of type "
              << model_type << " of view " << model_view << "." << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::writeModel (FeatureHashMapSingle &singlemap, std::ofstream &mfile)
  {
    std::vector<PointPairSystem<PointT> > systems;
    typename FeatureHashMapSingle::iterator it;
    typename std::vector<PointPairSystem<PointT> >::iterator it2;
    for (it = singlemap.begin (); it != singlemap.end (); ++it)
    {
      mfile << it->first.print () << ":";
      systems = it->second;
      for (it2 = systems.begin (); it2 != systems.end (); ++it2)
      {
        mfile << it2->print () << ",";
      }
      mfile << "\n";
    }
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::readModelsFromFile (std::istream &mfile)
  {
    #if FSR_HASHDESCRIPTION_VERBOSE
    int c = 0;
    #endif // FSR_HASHDESCRIPTION_VERBOSE
    while (readModel(mfile))
    {
      #if FSR_HASHDESCRIPTION_VERBOSE
      ++c;
      #endif // FSR_HASHDESCRIPTION_VERBOSE
    }
    m_ /= static_cast<int> (modelDiams_->size ());
    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::readModelsFromFile()]: " << c << " models read " << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE
  }

  template <typename PointT>
  bool FSRHashMapDescription<PointT>::readModel (std::istream &mfile)
  {
    std::string line;
    std::getline (mfile, line);

    if (line.compare("{") != 0)
    {
      return false;
    }

    std::getline (mfile, line); /// get model info

    std::stringstream ss (line);
    std::string value;

    /// read model name, type, and view
    std::getline (ss, value, ',');
    std::string mname = value;
    std::getline (ss, value, ',');
    int mtype = std::stoi (value);
    std::getline (ss, value, ',');
    int mview = std::stoi (value);
    std::getline (ss, value, ',');
    float mdiam = std::stof (value);
    modelDiams_->insert (mname, mtype, mview, mdiam);
    std::getline(ss, value, ',');
    int msize = std::stoi (value);
    modelSizes_->insert (mname, mtype, mview, msize);
    m_ += msize;

    #if FSR_HASHDESCRIPTION_DEBUG
    float tempf = 1.0f;
    if (!(modelDiams_->at (mname, mtype, mview, tempf)))
    {
      std::cout << "[fsr_or::FSRHashMapDescription::readModel()]: something went wrong with ObjectMap<float>::insert()!" << std::endl;
    }
    int tempi = 1;
    if (!(modelSizes_->at (mname, mtype, mview, tempi)))
    {
      std::cout << "[fsr_or::FSRHashMapDescription::readModel()]: something went wrong with ObjectMap<int>::insert()!" << std::endl;
    }
    #endif // FSR_HASHDESCRIPTION_DEBUG

    std::getline (mfile, line);
    FSRFeature feature;
    do
    {
      std::stringstream ss2 (line);

      /// read FSRFeature
      std::getline(ss2, value, ':');
      FSRFeature::read(value, feature);

      /// read point pairs
      std::vector<PointPairSystem<PointT> > systems;
      PointPairSystem<PointT> pps;
      std::getline (ss2, value, ',');
      while (PointPairSystem<PointT>::read (value, pps))
      {
        systems.push_back (pps);
        std::getline (ss2, value, ',');
      }

      H_->insert (feature, mname, mtype, mview, systems);

      #if FSR_HASHDESCRIPTION_DEBUG
      std::vector<PointPairSystem<PointT> > tempsys = std::vector<PointPairSystem<PointT> > ();
      if (!(H_->at (feature, mname, mtype, mview, tempsys)))
      {
        std::cout << "[fsr_or::FSRHashMapDescription::readModel()]: something went wrong with ObjectMap<std::vector<PointPairSystem<PointT> > >::insert()!" << std::endl;
      }
      #endif // FSR_HASHDESCRIPTION_DEBUG

      std::getline (mfile, line);
    }
    while (line.compare ("}") != 0);

    #if FSR_HASHDESCRIPTION_VERBOSE
    std::cout << "[fsr_or::FSRHashMapDescription::readModel()]: read model " << mname << " of type "
              << mtype << " of view " << mview << "." << std::endl;
    #endif // FSR_HASHDESCRIPTION_VERBOSE

    return true;
  }

  template <typename PointT>
  void FSRHashMapDescription<PointT>::trimFeaturesFromSingleTable (FeatureHashMapSingle &singlemap)
  {
    int sum = 0, target = 0, upper = 0, lower = 0, c = 0;
    typename FeatureHashMapSingle::iterator it;
    for (it = singlemap.begin (); it != singlemap.end (); ++it)
    {
      c = static_cast<int> (it->second.size ());
      sum += c;
      if (c > upper)
      {
        upper = c;
      }
    }
    target = static_cast<int> (ceil (K_ * sum));
    lower = upper - 1;

    while (sum > target)
    {
      for (it = singlemap.begin (); it != singlemap.end (); ++it)
      {
        c = static_cast<int> (it->second.size ());
        if (c > lower)
        {
          it->second.pop_back ();
          --sum;
        }
      }
      --upper;
      --lower;
    }
  }

  template <typename PointT>
  bool FSRHashMapDescription<PointT>::validateCloudForProcessing (CloudConstPtr &cloud)
  {
    std::vector<int> pt_indices;
    CloudPtr cloud_test (new Cloud ());
    pcl::removeNaNFromPointCloud (*cloud, *cloud_test, pt_indices);
    if (pt_indices.size() < 1000)
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

//template struct tbb::tbb_hash_compare<fsr_or::FSRFeature>;

template class fsr_or::FSRHashMapDescription<pcl::PointXYZ>;
template class fsr_or::FSRHashMapDescription<pcl::PointXYZRGBA>;
template class fsr_or::FSRHashMapDescription<pcl::PointXYZRGB>;
