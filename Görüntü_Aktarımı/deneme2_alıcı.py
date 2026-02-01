import cv2
import numpy as np
import socket

udp_port = 5000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", udp_port))

while True:
    data, _ = sock.recvfrom(65536)
    frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is not None:
        cv2.imshow('UDP Stream', frame)
    if cv2.waitKey(1) == 27: break