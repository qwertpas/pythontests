import socket
import time
import pandas as pd


class Test:
    port = 6008
    address = "127.0.0.1"
    attempts = 1000

    sSocket = socket.socket()
    cSocket = socket.socket()
    sSocket.bind((address, port))

    def __init__(self):
        self.sSocket.listen(1)
        self.cSocket.connect((self.address, self.port))
        c, null = self.sSocket.accept()
        i = 0
        latencyList = []
        while i < self.attempts:
            c.send(str(1000 * time.time()))
            latencyList.append(1000 * time.time() - float(self.cSocket.recv(1024)))
            i += 1
        print 'finished'
        print pd.DataFrame({'latency (ms)': latencyList}).describe()


if __name__ == '__main__':
    x = Test()