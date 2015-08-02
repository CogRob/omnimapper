#ifndef _FSR_TYPES
#define _FSR_TYPES

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <tbb/concurrent_hash_map.h>
#include <boost/unordered_map.hpp>
#include <Eigen/Dense>
#include <math.h>
#include <iostream>
#include <vector>
#include <string>
#include <utility>

#define FSR_TYPES_DEBUG 0
#define FSR_TYPES_VERBOSE 0

namespace fsr_or
{
  extern Eigen::IOFormat VectorDisplayFmt;
  extern Eigen::IOFormat VectorPrintFmt;
  extern Eigen::IOFormat MatrixPrintFmt;

  extern bool computeOPFFeature(const Eigen::Vector4f &pu, const Eigen::Vector4f &nu,
                                const Eigen::Vector4f &pv, const Eigen::Vector4f &nv,
                                float &f1, float &f2, float &f3, float &f4,
                                const float &d);
  template<typename T>
  struct HashCompare
  {
    static size_t hash(const T &key)
    {
      return boost::hash_value(key);
    }

    static bool equal(const T &key1, const T &key2)
    {
      return ( key1 == key2 );
    }
  };

  template <typename PointT>
  struct OrientedPoint
  {
    PointT p;
    pcl::Normal n;

    OrientedPoint () {}

    OrientedPoint (PointT p, pcl::Normal n)
    : p (p),
      n (n)
    {}
  };

  template <typename PointT>
  struct PointPair
  {
    OrientedPoint<PointT> pu;
    OrientedPoint<PointT> pv;

    PointPair () {}

    PointPair (OrientedPoint<PointT> u, OrientedPoint<PointT> v)
    : pu (u),
      pv (v)
    {}

    PointPair (PointT pu, pcl::Normal nu, PointT pv, pcl::Normal nv)
    : pu (OrientedPoint<PointT> (pu, nu)),
      pv (OrientedPoint<PointT> (pv, nv))
    {}

    inline std::string display ()
    {
      std::stringstream ss;
      ss << "[u: (" << this->pu.p.x << "," << this->pu.p.y << "," << this->pu.p.z << ")";
      ss << ",";
      ss << "(" << this->pu.n.normal_x << "," << this->pu.n.normal_y << "," << this->pu.n.normal_z << ")";
      ss << " and ";
      ss << "v: (" << this->pv.p.x << "," << this->pv.p.y << "," << this->pv.p.z << ")";
      ss << ",";
      ss << "(" << this->pv.n.normal_x << "," << this->pv.n.normal_y << "," << this->pv.n.normal_z << ")]";
      return ss.str ();
    }

    inline std::string print () const
    {
      std::stringstream ss;
      ss << "(";
      ss << this->pu.p.x << "," << this->pu.p.y << "," << this->pu.p.z << ",";
      ss << this->pu.n.normal_x << "," << this->pu.n.normal_y << "," << this->pu.n.normal_z;
      ss << "),(";
      ss << this->pv.p.x << "," << this->pv.p.y << "," << this->pv.p.z << ",";
      ss << this->pv.n.normal_x << "," << this->pv.n.normal_y << "," << this->pv.n.normal_z;
      ss << ")";
      return ss.str ();
    }

    static void read (std::string &sopp, PointPair<PointT> &opp)
    {
      std::stringstream ss (sopp);
      std::string value;

      std::getline (ss, value, '(');
      std::getline (ss, value, ','); opp.pu.p.x = std::stof (value);
      std::getline (ss, value, ','); opp.pu.p.y = std::stof (value);
      std::getline (ss, value, ','); opp.pu.p.z = std::stof (value);
      std::getline (ss, value, ','); opp.pu.n.normal_x = std::stof (value);
      std::getline (ss, value, ','); opp.pu.n.normal_y = std::stof (value);
      std::getline (ss, value, ')'); opp.pu.n.normal_z = std::stof (value);

      std::getline(ss, value, ',');

      std::getline(ss, value, '(');
      std::getline (ss, value, ','); opp.pv.p.x = std::stof (value);
      std::getline (ss, value, ','); opp.pv.p.y = std::stof (value);
      std::getline (ss, value, ','); opp.pv.p.z = std::stof (value);
      std::getline (ss, value, ','); opp.pv.n.normal_x = std::stof (value);
      std::getline (ss, value, ','); opp.pv.n.normal_y = std::stof (value);
      std::getline (ss, value, ')'); opp.pv.n.normal_z = std::stof (value);
    }
  };

  template <typename PointT>
  struct PointPairSystem
  {
    PointPair<PointT> opp;
    Eigen::Matrix4f F;

    PointPairSystem () {}

    PointPairSystem (PointPair<PointT> opp, Eigen::Matrix4f F)
    : opp (opp),
      F (F)
    {}

    inline std::string print () const
    {
      std::stringstream ss;
      ss << "{" << this->opp.print () << "," << this->F.format (MatrixPrintFmt) << "}";
      return ss.str();
    }

    static bool read (std::string &spps, PointPairSystem<PointT> &pps)
    {
      if (spps.size () < 16)
      {
        return false;
      }

      std::stringstream ss (spps);
      std::string value;

      std::getline (ss, value, '{');

      std::getline (ss, value, ',');
      PointPair<PointT>::read (value, pps.opp);

      std::getline (ss, value, '(');
      for (size_t i = 0; i < 15; i++)
      {
        std::getline (ss, value, ',');
        pps.F(i / 4, i % 4) = std::stof (value);
      }
      std::getline (ss, value, ')');
      pps.F(3, 3) = std::stof (value);

      return true;
    }
  };

  struct FSRFeature : public std::pair<int, std::pair<int,int> >
  {
    FSRFeature () {}

    /// f1 is constant
    FSRFeature (int f2, int f3, int f4)
    : std::pair<int, std::pair<int,int> > (f2, std::pair<int,int> (f3, f4))
    {}

    inline std::string print () const
    {
      std::stringstream ss;
      ss << this->first << "," << this->second.first << "," << this->second.second;
      return ss.str ();
    }

    static bool read (std::string &sf, FSRFeature &f)
    {
      std::stringstream ss (sf);
      std::string value;

      std::getline (ss, value, ',');
      f.first = std::stoi (value);
      std::getline (ss, value, ',');
      f.second.first = std::stoi (value);
      std::getline (ss, value);
      f.second.second = std::stoi (value);

      return true;
    }
  };

  template <typename PointT>
  bool computeF (const PointPair<PointT> &uv, Eigen::Matrix4f &F)
  {
    F = Eigen::Matrix4f::Zero();
    F(3,3) = 1.0f;
    /*
    F << 0,0,0,0,
         0,0,0,0,
         0,0,0,0,
         0,0,0,1;*/

    Eigen::Vector4f pu = uv.pu.p.getVector4fMap ();
    Eigen::Vector4f nu = uv.pu.n.getNormalVector4fMap ();
    Eigen::Vector4f pv = uv.pv.p.getVector4fMap ();
    Eigen::Vector4f nv = uv.pv.n.getNormalVector4fMap ();

    Eigen::Vector4f puv = pv - pu;
    Eigen::Vector4f nuv = nv + nu;

    Eigen::Vector4f pcrossn = puv.cross3 (nuv);
    Eigen::Vector4f pcrossncrossv = pcrossn.cross3 (puv);

    F.block<4,1>(0,0) = pcrossn/pcrossn.norm ();
    F.block<4,1>(0,1) = puv/puv.norm ();
    F.block<4,1>(0,2) = pcrossncrossv/pcrossncrossv.norm ();
    F.block<4,1>(0,3) = (pu + pv) / 2.0;

    return true;
  }

  template <typename PointT>
  double computeCloudResolution (typename pcl::PointCloud<PointT>::ConstPtr &cloud)
  {
    double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices (2);
	std::vector<float> squaredDistances (2);
	pcl::search::KdTree<PointT> tree;
	tree.setInputCloud (cloud);

	for (size_t i = 0; i < cloud->size (); ++i)
	{
		if (!pcl_isfinite ((*cloud)[i].x))
		{
		  continue;
		}

        nres = tree.nearestKSearch (i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt (squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
	{
	  resolution /= numberOfPoints;
	}

	return resolution;
  }

  struct OMKey : public std::pair<std::string, std::pair<int, int> >
  {
    OMKey () {}

    OMKey (std::string m, int t, int v)
    : std::pair<std::string, std::pair<int, int> > (m, std::pair<int, int> (t,v))
    {}
  };

  template <typename T>
  class ObjectMap : public boost::unordered_map<OMKey, T>
  {
    public:
      typedef boost::shared_ptr<ObjectMap> Ptr;
      typedef typename ObjectMap::iterator iterator;

      inline void insert_model (OMKey &mkey, T &val)
      {
        (*this)[mkey] = T (val);
      }

      inline iterator find_model (OMKey &mkey)
      {
        return this->find (mkey);
      }

      inline std::string objectFileName (OMKey &mkey) const
      {
        std::stringstream ss;
        /// database/model/model_#/model_type_#.pcd
        ss << "database/" << mkey.first << "/" << mkey.first << "_" << mkey.second.first << "/"
           << mkey.first << "_" << mkey.second.first << "_" << mkey.second.second << ".pcd";
        return ss.str ();
      }

      Ptr makeShared () { return Ptr (new ObjectMap (*this)); }
  };

  template <typename PointT>
  class FeatureHashMap : public boost::unordered_map<FSRFeature, ObjectMap<std::vector<PointPairSystem<PointT> > > >
  {
    public:
      typedef boost::shared_ptr<FeatureHashMap> Ptr;

      inline void insert_model (FSRFeature &f, OMKey &mkey, std::vector<PointPairSystem<PointT> > &syss)
      {
        ((*this)[f]).insert_model (mkey, syss);
      }

      inline typename ObjectMap<std::vector<PointPairSystem<PointT> > >::iterator find_model (FSRFeature &f, OMKey &mkey)
      {
        return ((*this)[f]).find_model (mkey);
      }

      inline typename ObjectMap<std::vector<PointPairSystem<PointT> > >::iterator begin_model (FSRFeature &f)
      {
        return ((*this)[f]).begin ();
      }

      inline typename ObjectMap<std::vector<PointPairSystem<PointT> > >::iterator end_model (FSRFeature &f)
      {
        return ((*this)[f]).end ();
      }

      Ptr makeShared () { return Ptr (new FeatureHashMap (*this)); }
  };
}

#endif
