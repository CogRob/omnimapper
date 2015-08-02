#include <fsr_threedorlib/fsr_types.h>

namespace fsr_or
{

  Eigen::IOFormat VectorDisplayFmt (-1, 1, ", ", ", ", "", "", "(", ")");
  Eigen::IOFormat VectorPrintFmt (-1, 1, ",", ",", "", "", "(", ")");
  Eigen::IOFormat MatrixPrintFmt (-1, 1, ",", ",", "", "", "(", ")");

  bool computeOPFFeature(const Eigen::Vector4f &pu, const Eigen::Vector4f &nu,
                         const Eigen::Vector4f &pv, const Eigen::Vector4f &nv,
                         float &f1, float &f2, float &f3, float &f4,
                         const float &d)
  {
    Eigen::Vector4f puv = pv - pu;
    f1 = puv.norm ();

    if (f1 == 0.0f)
    {
      #if FSR_TYPES_DEBUG
      std::cout << "[fsr_or::computeFeature()]: pu -> " << pu.format(VectorDisplayFmt) << " and pv -> "
                << pu.format(VectorDisplayFmt) << " are the same point." << std::endl;
      #endif // FSR_TYPES_DEBUG
      f1 = f2 = f3 = f4 = 0.0;
      return false;
    }
    else if (f1 > d)
    {
      #if FSR_TYPES_VERBOSE
      std::cout << "[fsr_or::computeFeature()]: pu -> " << pu.format(VectorDisplayFmt) << " and pv -> "
                << pu.format(VectorDisplayFmt) << " are too far apart." << std::endl;
      #endif // FSR_TYPES_VERBOSE
      f1 = f2 = f3 = f4 = 0.0;
      return false;
    }

    //Eigen::Vector4f nu_copy = nu, nv_copy = nv;
    float alpha_u = acos (fabs (nu.dot (puv)));
    float alpha_v = acos (fabs (nv.dot (-puv)));
    if (alpha_u < alpha_v)
    {
      f2 = acos (fabs (nu.dot (nv)));
      f3 = alpha_u;
      f4 = alpha_v;
    }
    else
    {
      /// switch pu and pv
      //nu_copy = nv;
      //nv_copy = nu;
      puv *= (-1);
      f2 = acos (fabs (nv.dot (nu)));
      f3 = acos (fabs (nv.dot (puv)));
      f4 = acos (fabs (nu.dot (-puv)));
    }
    return true;
    //return pcl::computePairFeature(pu, nu, pv, nv, f.first, f.second.first, f.second.second, f1);
  }
}
