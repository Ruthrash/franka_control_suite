#include <joint_publisher.h>

JointPublisher::JointPublisher(std::string port) : socket_(ctx_, zmq::socket_type::pub) {
    socket_.bind(port);
    port_ = port;
}

JointPublisher::JointPublisher(const JointPublisher& joint_listener) : socket_(ctx_, zmq::socket_type::pub)  {
    port_ = joint_listener.port_;
    socket_.bind(port_);
}

void JointPublisher::writeMessage(std::vector<double> jointAngles) {
    zmq::message_t jointAnglesMessage(jointAngles);
    socket_.send(jointAnglesMessage, zmq::send_flags::dontwait);
}