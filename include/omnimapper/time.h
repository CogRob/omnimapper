#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>

namespace omnimapper
{
 typedef boost::posix_time::ptime Time;

  boost::posix_time::ptime stamp2ptime (uint64_t stamp);

  /** \brief Gets a timestamp.  Allows flexibility, such as using the ros::Time system in place of the system clock
   */
  class GetTimeFunctor
  {
    public:
      virtual Time operator()() = 0;
  };
  
  class GetSystemTimeFunctor : public GetTimeFunctor
  {
    public:
      Time operator()() { return (boost::posix_time::microsec_clock::local_time ()); }
  };

  typedef boost::shared_ptr<omnimapper::GetTimeFunctor> GetTimeFunctorPtr;

}
