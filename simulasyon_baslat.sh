#!/bin/bash

echo "Simülasyon başlatılıyor..."
echo "Kullanılan parametre dosyası: ~/estuiha_gazebo/params/cessna.parm"
echo "ROS video server ve simülasyon başlatılacak."

# ROS ortam değişkenleri (IP adreslerini kendi ağınıza göre düzenleyin)
export ROS_MASTER_URI=http://192.168.1.100:11311
export ROS_IP=192.168.1.101

# --- Orijinal scriptinizin devamı buradan başlar ---
# ... existing code ...
# (Buraya orijinal .sh dosyanızın içeriğini ekleyin, hiçbir satırı eksiltmeden)
# ... existing code ...

# Her bir screen komutundan sonra bilgi mesajı
# Örnek:
echo "ArduPlane simülasyonu başlatıldı."
echo "Web video server başlatıldı."
echo "Simülasyon ortamı başlatıldı."

# Script sonunda tamamlandı mesajı
echo "Tüm işlemler tamamlandı. Simülasyon başlatıldı." 