import cv2
from flask import Flask, Response

app = Flask(__name__)
cap = cv2.VideoCapture(0)  # Kamera ID, gerekirse değiştir

def gen():
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        ret, jpeg = cv2.imencode('.jpg', frame)
        if ret:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host='0.0.0.0', port=8081)
