/*****************************************************************************/
/*! \file    Robot.cpp
 *  \author  LH
 *  \date    2011-05-26
 *
 *  \brief   Robot informations sent by the referee box
 *****************************************************************************/
#include "tku_libs/robocup_referee/robot.h"

using namespace std;

namespace robocup_referee
{
Robot::Robot()
{
  m_penalty = 0;
  m_secs_till_unpenalised = 0;
}

Robot::~Robot()
{
}

int Robot::getPenalty() const
{
  return m_penalty;
}

int Robot::getSecsTillUnpenalised() const
{
  return m_secs_till_unpenalised;
}

int Robot::getYellowCardCount() const
{
  return m_yellow_card_count;
}

int Robot::getRedCardCount() const
{
  return m_red_card_count;
}

/* Use a broadcasted message to update the Robot */
void Robot::update_from_message(char const* message, int numRobot)
{
  int d = Constants::nb_chars_by_robot * numRobot;  // decalage
  m_penalty = chars_to_int(message, d + 0, d + 1);
  m_secs_till_unpenalised = chars_to_int(message, d + 1, d + 2);
  m_yellow_card_count = chars_to_int(message, d + 2, d + 3);
  m_red_card_count = chars_to_int(message, d + 3, d + 4);
}

ostream& operator<<(ostream& flux, Robot const& r)
{
  flux << "\t\tpenalty : " << r.getPenalty() << endl;
  flux << "\t\tsecs_till_unpenalised : " << r.getSecsTillUnpenalised() << endl;
  flux << "\t\tyellow_card_count : " << r.getYellowCardCount() << endl;
  flux << "\t\tred_card_count : " << r.getRedCardCount() << endl;
  return flux;
}

}  // namespace robocup_referee
