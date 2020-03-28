#pragma once
//#include <omnimapper/omnimapper_base.h>

namespace omnimapper {
// Forward Declaration
class OmniMapperBase;

class MeasurementPlugin {
 private:
  OmniMapperBase* mapper_;

 public:
  MeasurementPlugin();
  virtual ~MeasurementPlugin();
  virtual bool addMeasurement();
};
}  // namespace omnimapper
