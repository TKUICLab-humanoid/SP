/*****************************************************************************/
/*! \file    Team.cpp
 *  \author  LH
 *  \date    2011-05-26
 *
 *  \brief   Team informations sent by the referee box
 *****************************************************************************/
#include "tku_libs/robocup_referee/team.h"

using namespace std;

namespace robocup_referee
{
Team::Team()
{
  m_team_number = 0;
  m_team_color = 0;
  m_score = 0;
}

Team::~Team()
{
}

int Team::getTeamNumber() const
{
  return m_team_number;
}

int Team::getTeamColor() const
{
  return m_team_color;
}

int Team::getScore() const
{
  return m_score;
}

int Team::getNbRobots() const
{
  return NB_ROBOTS;
}

const Robot& Team::getRobot(int robot) const
{
  return m_robots[robot];
}

/* Use a broadcasted message to update the Robot */
void Team::update_from_message(char const* message, int numTeam)
{
  int d = Constants::nb_chars_by_team * numTeam;  // decalage
  m_team_number = chars_to_int(message, d + 0, d + 1);
  m_team_color = chars_to_int(message, d + 1, d + 2);
  m_score = chars_to_int(message, d + 2, d + 3);
  for (int robot = 0; robot < NB_ROBOTS; robot++)
  {
    m_robots[robot].update_from_message(message + d + 260 + Constants::nb_chars_by_robot, robot);
  }
}

ostream& operator<<(ostream& flux, Team const& t)
{
  flux << '\t' << "team_number : " << t.getTeamNumber() << endl;
  flux << '\t' << "team_color : " << t.getTeamColor() << endl;
  flux << '\t' << "score : " << t.getScore() << endl;
  for (int robot = 0; robot < NB_ROBOTS; robot++)
  {
    flux << '\t' << "robot " << robot << std::endl;
    flux << t.getRobot(robot);
  }
  return flux;
}
}  // namespace robocup_referee
