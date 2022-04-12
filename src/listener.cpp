#pragma once

#include "listener.h"
#include <iostream>

Listener::Listener(CommsDataType dataType, std::string portId) : type(dataType), socket(ctx, zmq::socket_type::sub) {
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(portId);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = portId;
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
    readMessage();
    std::cout << "values size " << values.size() << std::endl;
}

Listener::Listener(const Listener& listener) : type(listener.type), socket(ctx, zmq::socket_type::sub){
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(listener.port);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = listener.port;
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
    readMessage();
    std::cout << "values size " << values.size() << std::endl;
}

void Listener::readMessage() {
    zmq::message_t message;
    socket.recv(&message);
 
    int numValues = message.size() / sizeof(double);
    assert(numValues == typeLengths[type]);
    std::cout << "num values " << numValues << std::endl;

    for(int i = 0; i < numValues; i++) {
        values[i] = *(reinterpret_cast<double*>(message.data()) + i);
        std::cout << values[i] << std::endl;
    }

    for(auto x : values)
        std::cout << "array vals " << x << std::endl;
}
