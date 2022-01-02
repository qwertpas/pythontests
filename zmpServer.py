import time
import zmq

port = "6006"
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.bind("tcp://*:%s" % port)

while True:
    socket.recv()
    socket.send(repr(1000 * time.time()))
    