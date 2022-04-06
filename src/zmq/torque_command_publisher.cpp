#include "zmq/torque_command_publisher.hpp"

TorqueCommandPublisher::TorqueCommandPublisher(std::string port) : socket_(ctx_, zmq::socket_type::pub) {
    socket_.bind(port);
    port_ = port;
}

TorqueCommandPublisher::TorqueCommandPublisher(const TorqueCommandPublisher& joint_listener) : socket_(ctx_, zmq::socket_type::pub)  {
    port_ = joint_listener.port_;
    socket_.bind(port_);
}

void TorqueCommandPublisher::writeMessage(const std::array<double, 7>& torqueCommands) {
    zmq::message_t torqueCommandsMessage(torqueCommands);
    socket_.send(torqueCommandsMessage, zmq::send_flags::dontwait);
}