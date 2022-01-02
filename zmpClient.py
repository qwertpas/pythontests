import time
import zmq

port = "6006"
context = zmq.Context()
socket = context.socket(zmq.PAIR)
socket.connect("tcp://127.0.0.1:%s" % port)

MYTIME = 1000 * time.time()
socket.send('')
print(float(socket.recv()) - MYTIME)
