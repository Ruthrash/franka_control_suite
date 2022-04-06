#!/usr/bin/env python
import zmq   
import sys 
import numpy as np 
import tf.transformations as tf_utils

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://127.0.0.1:2000")


for request in range(1,10):
  socket.send_string("HELLO")
  message = socket.recv()
  zmqPose = np.frombuffer(message).astype(np.float32)
  print(zmqPose)
  zmqPose = np.reshape(a=zmqPose, newshape=(4,4), order='F')
  print(zmqPose)
  zmq_position = zmqPose[:3,3]
  zmq_quat = tf_utils.quaternion_from_matrix(zmqPose)
  print(zmq_position, zmq_quat)