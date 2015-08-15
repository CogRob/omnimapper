#include <fsr_threedorlib/fsr_types.h>

namespace fsr_or
{

  Eigen::IOFormat VectorDisplayFmt (-1, 1, ", ", ", ", "", "", "(", ")");
  Eigen::IOFormat VectorPrintFmt (-1, 1, ",", ",", "", "", "", "");
  Eigen::IOFormat MatrixDisplayFmt (-1, 0, ",", ",", "", "", "[", "]");
  Eigen::IOFormat MatrixPrintFmt (-1, 1, ",", ",", "", "", "", "");

  bool computeOPFFeature(const Eigen::Vector4f &pu, const Eigen::Vector4f &nu,
                         const Eigen::Vector4f &pv, const Eigen::Vector4f &nv,
                         float &f1, float &f2, float &f3, float &f4,
                         const float &dminSq, const float &dmaxSq)
  {
    float dx = pv[0] - pu[0];
    float dy = pv[1] - pu[1];
    float dz = pv[2] - pu[2];
    f1 = dx*dx + dy*dy + dz*dz;

    if (f1 == 0.0f)
    {
      #if FSR_TYPES_DEBUG
      std::cout << "[fsr_or::computeFeature()]: pu -> " << pu.format(VectorDisplayFmt) << " and pv -> "
                << pu.format(VectorDisplayFmt) << " are the same point." << std::endl;
      #endif // FSR_TYPES_DEBUG
      f1 = f2 = f3 = f4 = 0.0;
      return false;
    }
    else if (f1 < dminSq || f1 > dmaxSq)
    {
      #if FSR_TYPES_VERBOSE
      std::cout << "[fsr_or::computeFeature()]: pu -> " << pu.format(VectorDisplayFmt) << " and pv -> "
                << pu.format(VectorDisplayFmt) << " are too far apart." << std::endl;
      #endif // FSR_TYPES_VERBOSE
      f2 = f3 = f4 = 0.0;
      return false;
    }

    f1 = sqrtf(f1);

    /// n_u dot (pv-pu)
    float alpha_u = acos (fabs (nu[0]*dx + nu[1]*dy + nu[2]*dz));
    /// n_u dot -(pv-pu)
    float alpha_v = acos (fabs (-nu[0]*dx - nu[1]*dy - nu[2]*dz));
    if (alpha_u < alpha_v)
    {
      f2 = acos (fabs (nu[0]*nv[0] + nu[1]*nv[1] + nu[2]*nv[2]));
      f3 = alpha_u;
      f4 = alpha_v;
    }
    else
    {
      /// switch pu and pv
      f2 = acos (fabs (nu[0]*nv[0] + nu[1]*nv[1] + nu[2]*nv[2]));
      f3 = alpha_v;
      f4 = alpha_u;
    }

    return true;
    //return pcl::computePairFeature(pu, nu, pv, nv, f.first, f.second.first, f.second.second, f1);
  }

  bool operator== (OMKey::Ptr const &om1, OMKey::Ptr const &om2)
  {
    return (*om1) == (*om2);
  }

}
