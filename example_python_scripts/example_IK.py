import zmq 


def main():
    context = zmq.Context() 
    command_pub_socket = context.socket(zmq.PUB)
    command_pub_socket.bind("tcp://127.0.0.1:2069")

    ee_state_sub_socket = context.socket(zmq.SUB)
    ee_state_sub_socket.setsockopt(zmq.CONFLATE, True)
    ee_state_sub_socket.connect("tcp://127.0.0.1:2096")
    ee_state_sub_socket.setsockopt(zmq.SUBSCRIBE, b'')

    message = self.joint_state_sub_socket.recv()
    ee_now = np.frombuffer(message).astype(np.float32)

