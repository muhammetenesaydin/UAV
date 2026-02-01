import cv2
import zmq
import numpy as np
import base64

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://SERVER_IP:5555")  # Sunucu IP'si ile değiştirin
socket.setsockopt_string(zmq.SUBSCRIBE, '')

cv2.namedWindow('Live Stream', cv2.WINDOW_NORMAL)

try:
    while True:
        # Görüntüyü al
        jpg_as_text = socket.recv_string()
        jpg_original = base64.b64decode(jpg_as_text)
        img = cv2.imdecode(np.frombuffer(jpg_original, dtype=np.uint8), cv2.IMREAD_COLOR)
        
        # Görüntüyü göster
        cv2.imshow('Live Stream', img)
        
        # Çıkış için 'q' tuşu
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    cv2.destroyAllWindows()
    socket.close()
    context.term()