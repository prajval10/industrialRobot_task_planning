#!/usr/bin/env python
import socket
HOST = "localhost"    # The remote host
PORT = 30003              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send ("get_inverse_kin((150,50,100,1,0,0))")
data = s.recv(4096)
s.close()
print ("Received", repr(data))