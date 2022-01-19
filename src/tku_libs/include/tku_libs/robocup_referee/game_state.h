/*****************************************************************************/
/*! \file    GameState.h
 *  \author  LH
 *  \date    2011-05-26
 *
 *  \brief   Game informations sent by the referee box
 *	This class is used to store easily informations about the game
 *****************************************************************************/
#pragma once

#include "tku_libs/robocup_referee/team.h"
#include "tku_libs/robocup_referee/time_stamp.h"

#define NB_TEAMS 2

namespace robocup_referee
{
class GameState
{
public:
  GameState();
  ~GameState();
  int getLastUpdate() const;
  int getStructVersion() const;
  int getGameTyp() const;
  int getNumPlayer() const;
  int getActualGameState() const;
  int getFirstHalf() const;
  int getKickOffTeam() const;
  int getSecGameState() const;
  int getDropInTeam() const;
  int getDropInTime() const;
  int getEstimatedSecs() const;
  int getSecondarySecs() const;
  int getSecondaryTeam() const;
  int getSecondaryMode() const;
  int getNbTeam() const;
  int getGameType() const;
  const Team& getTeam(int teamNumber) const;

  /*! \brief Update the robot from a referee box message
   * return true if there has been an update and false if
   * the message was discarded (invalid struct version) */
  bool update_from_message(char const* message);

  void show(std::ostream& flux) const;

private:
  TimeStamp m_last_game_state_update;
  int m_struct_version;
  int m_game_type;
  int m_num_player;
  int m_actual_game_state;
  int m_first_half;
  int m_kick_off_team;
  int m_sec_game_state;
  int m_drop_in_team;
  int m_drop_in_time;
  int m_estimated_secs;
  int m_secondary_secs;
  int m_secondary_team;
  int m_secondary_mode;
  Team m_team[NB_TEAMS];
};

std::ostream& operator<<(std::ostream& flux, GameState const* myGameState);

}  // namespace robocup_referee
