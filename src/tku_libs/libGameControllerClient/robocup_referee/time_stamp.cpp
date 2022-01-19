#include "tku_libs/robocup_referee/time_stamp.h"

#include <sys/time.h>

using namespace std;

namespace robocup_referee
{
/* Caution, with a 16 bit computer, the result will be restricted to 655 secs~ */
int hundredth_of_seconds()
{
  struct timeval mychrono;
  gettimeofday(&mychrono, NULL);
  int r = mychrono.tv_usec / 10000;
  r += 100 * mychrono.tv_sec;
  return r;
};

unsigned int TimeStamp::firstStamp = hundredth_of_seconds();

TimeStamp::TimeStamp()
{
  m_stamp = hundredth_of_seconds();
}

int TimeStamp::getStamp() const
{
  return m_stamp;
}

void TimeStamp::update()
{
  m_stamp = hundredth_of_seconds();
}

void TimeStamp::set_to(unsigned int new_time)
{
  m_stamp = new_time;
}

/* Return the elapsed time since last update in hundredth of seconds */
int TimeStamp::elapsed_time() const
{
  return hundredth_of_seconds() - m_stamp;
}

void TimeStamp::show(ostream& flux) const
{
  flux << "elapsed time : " << elapsed_time() << " (real time : " << m_stamp << ")";
}

ostream& operator<<(ostream& flux, TimeStamp const& myTimeStamp)
{
  myTimeStamp.show(flux);
  return flux;
}

}  // namespace robocup_referee
