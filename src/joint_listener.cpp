// #include "joint_listener.h"
// #include <iostream>
// #include <chrono>

// JointListener::JointListener(std::string port) : socket_(ctx_, zmq::socket_type::sub) {
//     int confl = 1;
//     socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
//     socket_.connect(port);
//     socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
//     port_ = port;
//     // for(int i = 0; i < 7; i++) 
//     //     jointAngles[i] = 0;
//     // readMessage();
// }

// JointListener::JointListener(const JointListener& joint_listener) : socket_(ctx_, zmq::socket_type::sub){
//     int confl = 1;
//     socket_.setsockopt(ZMQ_CONFLATE, &confl, sizeof(int));
//     socket_.connect(joint_listener.port_);
//     socket_.setsockopt(ZMQ_SUBSCRIBE, "", 0);
//     port_ = joint_listener.port_;
//     // for(int i = 0; i < 7; i++) 
//     //     jointAngles[i] = 0;
//     // readMessage();
// }

// void JointListener::readMessage() {
//     zmq::message_t jointAnglesMessage;
//     socket_.recv(&jointAnglesMessage);
//     int numValues = jointAnglesMessage.size() / sizeof(double);
//     assert(numValues == 7);
//     // std::cout << "after assert  " << std::endl;

//     for(int i = 0; i < numValues; i++) {
//         std::cout << "num values " << i;
//         std::cout << " pointer dereference " << *(reinterpret_cast<double*>(jointAnglesMessage.data()) + i);
//         jointAngles[i] = *(reinterpret_cast<double*>(jointAnglesMessage.data()) + i);
//         std::cout << " " << jointAngles[i] << std::endl;
//     }
// }
