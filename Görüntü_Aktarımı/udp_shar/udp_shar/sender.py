import cv2
import socket
import pickle
import struct
import threading
import time

def get_local_ip():
    """Yerel IP adresini alır"""
    try:
        # Google DNS'e bağlanarak yerel IP'yi bulur
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except:
        return "127.0.0.1"

def send_data_in_chunks(socket_obj, data, address, chunk_size=1024):
    """Veriyi küçük parçalar halinde gönderir"""
    try:
        # Toplam veri boyutunu gönder
        total_size = len(data)
        socket_obj.sendto(struct.pack("!I", total_size), address)
        
        # Veriyi parçalara böl ve gönder
        chunks_sent = 0
        for i in range(0, total_size, chunk_size):
            chunk = data[i:i + chunk_size]
            socket_obj.sendto(chunk, address)
            chunks_sent += 1
            
        return True
    except Exception as e:
        print(f"Veri gönderme hatası: {e}")
        return False

def main():
    # UDP socket oluştur
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Yerel IP ve port
    host_ip = get_local_ip()
    port = 9999
    
    # Socket'i bind et
    server_socket.bind((host_ip, port))
    
    print(f"UDP Server başlatıldı")
    print(f"IP Adresi: {host_ip}")
    print(f"Port: {port}")
    print("Kameradan görüntü alınıyor ve gönderiliyor...")
    print("Çıkmak için 'q' tuşuna basın\n")
    
    # Kamerayı başlat
    cap = cv2.VideoCapture(0)
    
    # Görüntü kalitesini ayarla (boyutu küçültüyoruz)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_FPS, 20)
    
    client_address = None
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kamera görüntüsü alınamadı!")
            break
        
        # Görüntüyü daha fazla sıkıştır
        _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        data = buffer.tobytes()
        
        # IP ve port bilgisini görüntü üzerine yaz
        cv2.putText(frame, f"IP: {host_ip}:{port}", (10, 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(frame, "UDP Sender - Gonderiliyor...", (10, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        if client_address:
            cv2.putText(frame, f"Bagli: {client_address}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        # Görüntüyü göster
        cv2.imshow('UDP Sender - Kamera Görüntüsü', frame)
        
        # Eğer client adresi varsa, veriyi gönder
        if client_address:
            frame_count += 1
            print(f"Frame {frame_count} gönderiliyor... (Boyut: {len(data)} bytes)")
            success = send_data_in_chunks(server_socket, data, client_address)
            if not success:
                print("Gönderme hatası!")
        
        # İstemci bekleme (non-blocking)
        server_socket.settimeout(0.001)  # 1ms timeout
        try:
            message, addr = server_socket.recvfrom(1024)
            if message == b"REQUEST":
                client_address = addr
                print(f"İstemci bağlandı: {addr}")
        except socket.timeout:
            pass
        except Exception as e:
            pass
        
        # 'q' tuşuna basılırsa çık
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        # FPS kontrolü için kısa bekleme
        time.sleep(0.05)  # ~20 FPS
    
    # Temizleme
    cap.release()
    cv2.destroyAllWindows()
    server_socket.close()
    print("UDP Sender kapatıldı.")

if __name__ == "__main__":
    main() 