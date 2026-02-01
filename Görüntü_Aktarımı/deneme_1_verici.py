import cv2
import zmq
import base64
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://*:5555")  # Tüm ağ arayüzlerinde yayın yap

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Çözünürlük ayarı
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)  # FPS ayarı

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        # JPEG sıkıştırma (boyutu küçültmek için)
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        
        # Base64 kodlama
        jpg_as_text = base64.b64encode(buffer)
        socket.send(jpg_as_text)
        
        # Küçük bir bekleme (CPU yükünü azaltmak için)
        time.sleep(0.01)
        
finally:
    cap.release()
    socket.close()
    context.term()