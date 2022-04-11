#include <publisher.h>

Publisher::Publisher(std::string port) : socket(ctx, zmq::socket_type::pub) {
    socket.bind(port);
    port = port;
}

Publisher::Publisher(const Publisher& publisher) : socket(ctx, zmq::socket_type::pub) {
    port = publisher.port;
    socket.bind(port);
}

void Publisher::writeMessage(const std::vector<double>& data) {
    zmq::message_t message(data);
    socket.send(message, zmq::send_flags::dontwait);
}