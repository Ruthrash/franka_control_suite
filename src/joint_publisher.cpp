#include <joint_publisher.h>
#include <iostream>

JointPublisher::JointPublisher(std::string port) : socket_(ctx_, zmq::socket_type::pub) {
    socket_.bind(port);
    port_ = port;
}

JointPublisher::JointPublisher(const JointPublisher& joint_listener) : socket_(ctx_, zmq::socket_type::pub)  {
    port_ = joint_listener.port_;
    socket_.bind(port_);
}

void JointPublisher::writeMessage(const std::array<double, 14>& jointAngles) {
    zmq::message_t jointAnglesMessage(jointAngles);
    // std::cout << "sending to worksation" << std::endl;
    // for(int i = 0; i < 9; i++) 
        // std::cout << jointAngles[i] << std::endl;
    socket_.send(jointAnglesMessage, zmq::send_flags::dontwait);
    // std::cout << "sent" << std::endl;
}