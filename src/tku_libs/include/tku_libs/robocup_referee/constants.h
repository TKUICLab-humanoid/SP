/*****************************************************************************/
/*! \file    Constants.h
 *  \author  LH
 *  \date    2011-05-26
 *
 *  \brief   Useful informations shared by the robot
 *	By centralizing all constants in this file, it's easy to change some
 *	parameters if specifications change.
 *****************************************************************************/
#pragma once

#include <string>

// #include <hl_monitoring/field.h>

namespace robocup_referee
{
class Constants
{
public:
  /// Containing all the dimensions of the field
  // static hl_monitoring::Field field;

  /* Headers informations */
  static const int header_length = 4;
  static const char* game_state_header;
  /* Chars informations */
  static const int nb_chars_by_robot;
  static const int nb_chars_by_team;

  /* GAME INFORMATIONS */
  /* Game */
  static const int nb_teams = 2;
  static const int nb_robots_by_team = 5;

  /* GAMESTATE INFORMATIONS */

  /* Team colors */
  static const int TEAM_CYAN = 0;
  static const int TEAM_MAGENTA = 1;
  static const int TEAM_BLUE = 0;
  static const int TEAM_RED = 1;
  static const int DROPBALL = 2;

  static const int GOAL_BLUE = 0;
  static const int GOAL_YELLOW = 1;

  /* Game states */
  static const int STATE_INITIAL = 0;
  static const int STATE_READY = 1;
  static const int STATE_SET = 2;
  static const int STATE_PLAYING = 3;
  static const int STATE_FINISHED = 4;

  /* Secondary game states */
  static const int STATE2_NORMAL = 0;
  static const int STATE2_PENALTYSHOOT = 1;
  static const int STATE2_OVERTIME = 2;
  static const int STATE2_TIMEOUT = 3;
  static const int STATE2_DIRECT_FREE_KICK = 4;
  static const int STATE2_INDIRECT_FREE_KICK = 5;
  static const int STATE2_PENALTY_KICK = 6;
  static const int STATE2_CORNER_KICK = 7;
  static const int STATE2_GOAL_KICK = 8;
  static const int STATE2_THROW_IN = 9;

  /* Penalties */
  static const short PENALTY_NONE = 0;
  static const short PENALTY_SUBSTITUTE = 14;

  /* DURATIONS (in seconds) */
  /* Game informations */
  static const int PERIOD_DURATION = 600;              // Two Periods
  static const int HALFTIME_DURATION = 300;            // Maximum
  static const int ADDITIONNAL_PERIOD_DURATION = 300;  // Two again

  static const int TIMEOUT_DURATION = 120;

  /* Organization */
  static const int TIME_AFTER_PLAY_BEFORE_REAL_START = 10;  // For the team which isn't doing the kick-off
  static const int INTERVAL_SET_PLAY = 10;
  static const int INTERVAL_GOAL_SET = 30;  // After a goal, autonomous robots have 30 seconds to place well

  /* Rules */
  static const int HOLDING_LIMIT = 1;             // Field Players aren't allowed to keep the ball more than 1 seconds
  static const int GOALKEEPER_HOLDING_LIMIT = 5;  // GoalKeeper can hold the Ball in it's hands for 4 seconds
  static const int GOAL_AREA_LIMIT =
      10;  // None team is allowed to have two players in the goal area for more than 10 seconds

  static const int REMOVAL_PENALTY_DURATION = 30;

  /* PENALTY KICK */
  static const int MAX_DISTANCE_TO_PENALTY_MARK = 20;  // cm
  static const int PENALTY_DURATION = 60;              // secs
};
}  // namespace robocup_referee
