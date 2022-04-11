#pragma once

#include "listener.h"

Listener::Listener(CommsDataType type, std::string port) : type(type), socket(ctx, zmq::socket_type::sub) {
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(port);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = port;
    readMessage();
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
}

Listener::Listener(const Listener& listener) : type(listener.type), socket(ctx, zmq::socket_type::sub){
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(listener.port);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = listener.port;
    readMessage();
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
}

void Listener::readMessage() {
    zmq::message_t message;
    socket.recv(&message);
 
    int numValues = message.size() / sizeof(double);
    assert(numValues == typeLengths[type]);

    for(int i = 0; i < numValues; i++) 
        values[i] = *(reinterpret_cast<double*>(message.data()) + i);
}
