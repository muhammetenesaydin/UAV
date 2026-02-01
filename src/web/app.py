from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import threading
import time
import json
from src.core.mavlink_connection import MavlinkConnection
from src.common.config import Config

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# MAVLink Global Nesnesi
uav_conn = MavlinkConnection(Config.CONNECTION_STRING)

@app.route('/')
def index():
    return render_template('index.html')

def telemetry_stream():
    """Arka planda MAVLink verilerini okur ve web arayüzüne basar."""
    uav_conn.connect()
    while True:
        data = uav_conn.get_telemetry()
        socketio.emit('telemetry', data)
        socketio.sleep(0.1) # 10Hz veri akışı

@socketio.on('command')
def handle_command(cmd):
    """Web arayüzünden gelen komutları işler."""
    print(f"Alınan Komut: {cmd}")
    if cmd == 'takeoff':
        uav_conn.set_mode('GUIDED')
        uav_conn.arm()
        # Takeoff komutu burada controller üzerinden çağrılabilir
    elif cmd == 'rtl':
        uav_conn.set_mode('RTL')

if __name__ == '__main__':
    # Telemetri thread'ini başlat
    socketio.start_background_task(telemetry_stream)
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
