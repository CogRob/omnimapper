#pragma once

#include <gtsam/nonlinear/Values.h>

namespace omnimapper
{
  class OutputPlugin
  {
    public:
      virtual void update (boost::shared_ptr<gtsam::Values>& vis_values) = 0;
  };
  
}
