import socket

UDP_IP = "192.168.1.206"
UDP_PORT = 5002
b = bytearray(2)
b[0] = pow(2, 3) + 1
b[1] = 1 + 16
MESSAGE = buffer(b)

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE

sock = socket.socket(socket.AF_INET,  # Internet
                     socket.SOCK_DGRAM)  # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
