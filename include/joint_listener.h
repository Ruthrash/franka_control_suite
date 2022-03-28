#include <zmq.hpp>
#include <vector>

class JointListener {
public:
    JointListener(std::string port);
    JointListener(const JointListener& listener);
    void readMessage();

    std::vector<double> jointAngles;
    zmq::context_t ctx_;
    zmq::socket_t socket_;
    bool messageChanged_;
    std::string port_;
};
