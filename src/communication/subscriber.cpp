#include <math.h>

#include "communication/subscriber.h"
#include <iostream>


Subscriber::Subscriber(CommsDataType dataType, std::string portId) : type(dataType), socket(ctx, zmq::socket_type::sub) {
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(portId);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = portId;
    // values.push_back(0);
    // values.push_back(-M_PI_4);
    // values.push_back(0);
    // values.push_back(-3 * M_PI_4);
    // values.push_back(0);
    // values.push_back(M_PI_2);
    // values.push_back(M_PI_4);
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
}

Subscriber::Subscriber(const Subscriber& subscriber) : type(subscriber.type), socket(ctx, zmq::socket_type::sub){
    int confl = 1;
    socket.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
    socket.connect(subscriber.port);
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);
    port = subscriber.port;
    for(size_t i = 0; i < typeLengths[type]; i++)
        values.push_back(0.0);
}

void Subscriber::readMessage() {
    zmq::message_t message;
    socket.recv(&message);
 
    int numValues = message.size() / sizeof(double);
    assert(numValues == typeLengths[type]);
    
    std::lock_guard<std::mutex> guard(accessValuesMutex);
    for(int i = 0; i < numValues; i++)
        values[i] = *(reinterpret_cast<double*>(message.data()) + i);
}

void Subscriber::readValues(std::vector<double>& output) {
    std::lock_guard<std::mutex> guard(accessValuesMutex);
    if(output.size() != values.size()) 
        output.resize(values.size());

    std::cout << "received commands: ";
    for(size_t i = 0; i < values.size(); i++) {
        output[i] = values[i];
        std::cout << output[i] << " ";
    }
    std::cout << std::endl;

}

void Subscriber::setDataType(CommsDataType dataType) {
    type = dataType;
    values.resize(typeLengths[dataType]);
    for(size_t i = 0; i < typeLengths[type]; i++)
        values[i] = 0.0;
}

double Subscriber::readGripperCommands() {
    std::lock_guard<std::mutex> guard(accessValuesMutex);
    double finger1 = values.end()[-2];
    double finger2 = values.end()[-1];

    return finger1 + finger2;
}