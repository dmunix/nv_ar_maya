#include "MayaConnection.h"

#include <string>

#include "opencv2/core/core.hpp"


bool MayaConnection::connect() 
{
    try 
    {
        internal_socket.connect("localhost", 5055);
        connected = true;
        printf("Connected!\n");
    } catch (int e)
    {
        connected = false;
        printf("Connection failed! error code: %d\n", e);
    }
    return connected;
}


bool MayaConnection::send(const std::string& cmd)
{
    try {
        if(internal_socket.send(cmd) < cmd.length()) {
            std::cout << "full message not sent!" << std::endl;
        }
        return true;
    } catch (int e) {
        connect();
        printf("Failed to send msg! error code: %d\n", e);
        return false;
    }
    
    return true;
}
