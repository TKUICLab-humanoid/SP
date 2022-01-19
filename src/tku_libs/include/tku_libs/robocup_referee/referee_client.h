#pragma once

#include <string>
#include <thread>

#include "tku_libs/robocup_referee/game_state.h"
#include "tku_libs/sockets/udp_broadcast.h"

namespace robocup_referee
{
class RefereeClient
{
private:
  GameState _gamedata;

protected:
  std::thread* thread;
  int _portNo, _portSend;

public:
  static const int protocolVersion = 12;
  static const int protocolReturnVersion = 2;

  RefereeClient(int portNo = 3838, int portSend = 3939) : _portNo(portNo), _portSend(portSend){};
  virtual ~RefereeClient(){};

  void start();
  virtual void execute(void);
  virtual bool isIPValid(std::string ip);
  void setState(uint8_t teamId, uint8_t myId, uint8_t message);
  uint8_t _myTeamId;
  uint8_t _myId;
  uint8_t _message;

  GameState& getGameState();
};

class RefereeClientInstance : public RefereeClient
{
  public:
    RefereeClientInstance(int portNo = 3838, int portSend = 3939);
    ~RefereeClientInstance();
    static RefereeClientInstance* getInstance(int portNo = 3838, int portSend = 3939);
    static void deleteInstance();

  private:
    static RefereeClientInstance* m_pInstance;
};

}  // namespace robocup_referee
