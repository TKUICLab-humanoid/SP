/*****************************************************************************/
/*! \file    TimeStamp.h
 *  \author  LH
 *  \date    2011-05-30
 *
 *  \brief   Timer counting hundredth of seconds
 *	This class is used to store time informations on data updates
 *	This timer can only count until ~650 seconds on a 16 bit computer.
 *	The implemented methob use the gettimeofday() function, all the robots
 *	must have the same time if
 *****************************************************************************/
#pragma once

#include <iostream>

namespace robocup_referee
{
class TimeStamp
{
public:
  TimeStamp();

  /*! \brief Return the stamp without any modification */
  int getStamp() const;

  /*! \brief Change the TimeStamp to actual time */
  void update();

  /*! \brief Set the TimeStamp to the specified time */
  void set_to(unsigned int new_time);

  /*! \brief Return the elapsed time since last update in hundredth of seconds 自上次更新以來經過的時間 */
  int elapsed_time() const;

private:
  unsigned int m_stamp;            // This stamp is not modified
  unsigned static int firstStamp;  // Reference time

  void show(std::ostream& flux) const;

  friend std::ostream& operator<<(std::ostream& flux, TimeStamp const& myTimeStamp);
};

}  // namespace robocup_referee
