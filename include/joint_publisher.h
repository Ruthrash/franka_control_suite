#include <zmq.hpp>
#include <vector>

class JointPublisher {
public:
    JointPublisher(std::string port);
    JointPublisher(const JointPublisher& publisher);
    void writeMessage(std::vector<double> jointAngles);

    zmq::context_t ctx_;
    zmq::socket_t socket_;
    std::string port_;
};