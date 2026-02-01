from flask import Flask, Response
import cv2

app = Flask(__name__)

def generate_frames():
    camera = cv2.VideoCapture(1)  # Kamera nesnesini fonksiyon içinde oluştur
    
    # Kamera ayarları
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    while True:
        success, frame = camera.read()
        if not success:
            print("Kameradan görüntü alınamıyor!")  # Hata mesajı ekleyin
            break
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return """
    <html>
    <head>
        <title>Kamera Yayını</title>
    </head>
    <body>
        <h1>Kamera Yayını</h1>
        <img src="/video_feed" width="640" height="480">
    </body>
    </html>
    """

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)  # Tüm ağ arabirimlerini dinle