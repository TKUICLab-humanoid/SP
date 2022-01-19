/*****************************************************************************/
/*! \file    Constants.cpp
 *  \author  LH
 *  \date    2011-05-26
 *
 *  \brief   Useful informations shared by the robot
 *****************************************************************************/
#include "tku_libs/robocup_referee/constants.h"

namespace robocup_referee
{
// hl_monitoring::Field Constants::field;

/* Classes and messages size */
const int Constants::nb_chars_by_robot = 4;
const int Constants::nb_chars_by_team = 260 + Constants::nb_chars_by_robot * 12;
// The referee box works with up to 11 robots, plus the coach

/* Headers */
const char* Constants::game_state_header = "RGme";

}  // namespace robocup_referee
