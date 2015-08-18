#include <fsr_threedorlib/fsr_types.h>

namespace fsr_or
{

  Eigen::IOFormat VectorDisplayFmt (-1, 1, ", ", ", ", "", "", "(", ")");
  Eigen::IOFormat VectorPrintFmt (-1, 1, ",", ",", "", "", "", "");
  Eigen::IOFormat MatrixDisplayFmt (-1, 0, ",", ",", "", "", "[", "]");
  Eigen::IOFormat MatrixPrintFmt (-1, 1, ",", ",", "", "", "", "");

  bool operator== (OMKey::Ptr const &om1, OMKey::Ptr const &om2)
  {
    return (*om1) == (*om2);
  }

}
