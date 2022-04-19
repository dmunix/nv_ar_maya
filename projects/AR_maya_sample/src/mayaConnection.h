#ifndef __MAYACONNECTION_H
#define __MAYACONNECTION_H
#include <vector>
#include <iostream>
#include "Socket.h"


class MayaConnection
{
public:
        
    MayaConnection() {};
    virtual ~MayaConnection() {};

    bool connect();
    bool send(const std::string&);
    bool isConnected() const { return connected; };

private:

    Socket internal_socket;
    bool connected;
};

#endif 	/* MAYACONNECTION_H */
