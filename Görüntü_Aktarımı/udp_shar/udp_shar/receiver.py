import cv2
import socket
import struct
import numpy as np
import time

def receive_data_in_chunks(socket_obj, timeout=5.0):
    """Parçalar halinde gelen veriyi toplar"""
    try:
        socket_obj.settimeout(timeout)
        
        # İlk olarak toplam veri boyutunu al
        size_data, addr = socket_obj.recvfrom(4)
        total_size = struct.unpack("!I", size_data)[0]
        
        print(f"Veri alınıyor... Toplam boyut: {total_size} bytes")
        
        # Veriyi topla
        received_data = b""
        while len(received_data) < total_size:
            chunk, addr = socket_obj.recvfrom(1024)
            received_data += chunk
            
            # İlerleme göster
            progress = (len(received_data) / total_size) * 100
            print(f"\rİlerleme: {progress:.1f}%", end="", flush=True)
        
        print()  # Yeni satır
        return received_data
        
    except socket.timeout:
        print("Veri alma zaman aşımı!")
        return None
    except Exception as e:
        print(f"Veri alma hatası: {e}")
        return None

def main():
    print("UDP Receiver - Görüntü Alıcısı")
    print("="*40)
    
    # Kullanıcıdan IP ve port bilgisini al
    while True:
        try:
            sender_ip = input("Sender IP adresini girin: ").strip()
            sender_port = int(input("Sender Port numarasını girin: ").strip())
            break
        except ValueError:
            print("Geçersiz port numarası! Tekrar deneyin.")
        except KeyboardInterrupt:
            print("\nProgram kapatılıyor...")
            return
    
    # UDP socket oluştur
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print(f"\n{sender_ip}:{sender_port} adresine bağlanılıyor...")
    print("Görüntü alınmaya başlanıyor...")
    print("Çıkmak için 'q' tuşuna basın\n")
    
    # Sender'a bağlantı isteği gönder
    try:
        client_socket.sendto(b"REQUEST", (sender_ip, sender_port))
        print("Bağlantı isteği gönderildi...")
    except Exception as e:
        print(f"Bağlantı hatası: {e}")
        client_socket.close()
        return
    
    frame_count = 0
    last_time = time.time()
    
    while True:
        try:
            # Veriyi al
            data = receive_data_in_chunks(client_socket, timeout=3.0)
            
            if data is None:
                print("Veri alınamadı, yeniden deneniyor...")
                # Yeniden bağlantı isteği gönder
                client_socket.sendto(b"REQUEST", (sender_ip, sender_port))
                continue
            
            # Görüntüyü çöz
            frame_buffer = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(frame_buffer, cv2.IMREAD_COLOR)
            
            if frame is None:
                print("Görüntü çözülemedi!")
                continue
            
            frame_count += 1
            current_time = time.time()
            
            # FPS hesapla
            if current_time - last_time >= 1.0:
                fps = frame_count / (current_time - last_time)
                print(f"FPS: {fps:.1f}")
                frame_count = 0
                last_time = current_time
            
            # Alıcı bilgisini görüntü üzerine yaz
            cv2.putText(frame, f"Alici: {sender_ip}:{sender_port}", (10, frame.shape[0] - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            cv2.putText(frame, "UDP Receiver - Alinan Goruntu", (10, frame.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
            
            # Görüntüyü göster
            cv2.imshow('UDP Receiver - Alınan Görüntü', frame)
            
            # 'q' tuşuna basılırsa çık
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                
        except KeyboardInterrupt:
            print("\nKullanıcı tarafından durduruldu...")
            break
        except Exception as e:
            print(f"Beklenmeyen hata: {e}")
            print("Yeniden bağlanmaya çalışıyor...")
            # Yeniden bağlantı isteği gönder
            try:
                client_socket.sendto(b"REQUEST", (sender_ip, sender_port))
            except:
                pass
            continue
    
    # Temizleme
    cv2.destroyAllWindows()
    client_socket.close()
    print("UDP Receiver kapatıldı.")

if __name__ == "__main__":
    main()