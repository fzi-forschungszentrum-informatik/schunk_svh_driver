#include "WsBroadcaster.h"

#include <icl_core_logging/Logging.h>

namespace icl_comm {
namespace websocket {

DECLARE_LOG_STREAM(WebSocket);
REGISTER_LOG_STREAM(WebSocket);
 
using icl_core::logging::endl;

bool WsBroadcaster::sendState()
{
  //std::cout << "JSON MESSAGE: " << robot.getStateJSON() << std::endl;
  if (! socket)
  {
    try
    {
      socket.reset(new icl_comm::websocket::UnixSocket(m_path));
    }
    catch ( icl_comm::websocket::SocketException e )
    {
      reset_error_counter++;
      if(reset_error_counter == 1)
      {
        LOGGING_INFO_C(WebSocket, WsBroadcaster, "First Error in initializing Unix socket, will try again on next send: " << e.what() << endl);
      }
      if(reset_error_counter % 100 == 0)
      {
        LOGGING_INFO_C(WebSocket, WsBroadcaster, "Error appeared "<< reset_error_counter << " times while initializing Unix socket, still trying again on next send: " << e.what() << endl);
      }
      return false;
    }
  }

  // Send out the state as JSON encoded String
  if (!socket->send_raw_message(robot->getStateJSON()))
  {
    std::cout << "Socket send failed, reconnecting" << std::endl;
    try {
      socket->reconnect();
    } catch (std::exception & e) {
      send_error_counter++;
      if(send_error_counter == 1)
      {
        LOGGING_INFO_C(WebSocket, WsBroadcaster, "First Error in sending to Unix socket, it was reset and we will try again on next send: " << e.what() << endl);
      }
      if(send_error_counter % 100 == 0)
      {
        LOGGING_INFO_C(WebSocket, WsBroadcaster, "Error appeared "<< send_error_counter << " times while sending to Unix socket, it was reset and we will still try again on next send: " << e.what() << endl);
      }
    }
    return false;
  }
    
  return true;
}

}
}
