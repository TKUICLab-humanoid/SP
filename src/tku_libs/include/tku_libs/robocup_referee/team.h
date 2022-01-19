/*****************************************************************************/
/*! \file    Team.h
 *  \author  LH
 *  \date    2011-05-26
 *
 *  \brief   Team informations sent by the referee box
 *	This class is used to store the team informations
 *****************************************************************************/
#pragma once

#include "tku_libs/robocup_referee/robot.h"

#define NB_ROBOTS 6

namespace robocup_referee
{
class Team
{
public:
  Team();
  ~Team();

  int getTeamNumber() const;
  int getTeamColor() const;
  int getScore() const;
  int getNbRobots() const;
  const Robot& getRobot(int robot) const;

  /*! \brief Update the robot from a referee box message */
  void update_from_message(char const* message, int numTeam);

private:
  int m_team_number;
  int m_team_color;
  int m_score;
  Robot m_robots[NB_ROBOTS];
};

std::ostream& operator<<(std::ostream& flux, Team const& myTeam);

}  // namespace robocup_referee
