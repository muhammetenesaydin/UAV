# TEKNOFEST 2026 Savaşan İHA & Kamikaze Yer Kontrol İstasyonu

Bu proje, TEKNOFEST 2026 Savaşan İHA yarışması için geliştirilmiş, **SOLID** prensiplerine dayalı, modüler ve yüksek performanslı bir otonom kontrol sistemidir.

## 🚀 Öne Çıkan Özellikler

- **Modüler Mimari:** Algılama, Planlama ve Kontrol katmanları birbirinden bağımsızdır (Decoupled).
- **Gelişmiş Algılama:** YOLOv8 tabanlı hava aracı tespiti ve çoklu filtreleme (CLAHE, Adaptive Thresh) ile QR kod okuma.
- **Dinamik Görev Yönetimi:** Sonlu Durum Makinesi (State Machine) ile kalkış, arama, kilitlenme ve dalış fazları.
- **Modern GCS:** PyQt5 ile şık ve karanlık mod destekli yer kontrol arayüzü.

## 📂 Proje Yapısı

```text
uavGCS-main/
├── src/
│   ├── web/            # Profesyonel Web C2 Arayüzü (Flask + Socket.IO)
│   ├── core/           # MAVLink haberleşme ve arayüzler
│   ├── perception/     # YOLO ve QR kod işleme algoritmaları
│   ├── control/        # İHA manevra komutları
│   ├── mission/        # Otonom görev durum makinesi
│   └── main.py         # Sistemin ana giriş noktası
├── docs/               # Şartname ve raporlar
└── archive/            # Eski kodlar ve yedekler
```

## 🛠️ Kurulum

1. Bağımlılıkları yükleyin:
   ```bash
   pip install -r requirements.txt
   ```

2. Simülasyonu başlatın (bkz. Test Bölümü).

3. Web Arayüzünü çalıştırın:
   ```bash
   python3 -m src.web.app
   ```
   Ardından tarayıcınızda `http://localhost:5000` adresine gidin.

## 🧪 Test ve Simülasyon

Bu yazılımı gerçek bir İHA olmadan **ArduPilot SITL** ve **Gazebo** kullanarak test edebilirsiniz:

1. **ArduPilot SITL Kurulumu:** ArduPilot dünyasında yazılımı simüle etmek için kullanılır.
   ```bash
   cd ~/ardupilot/ArduPlane
   sim_vehicle.py -v ArduPlane --console --map
   ```

2. **Gazebo Entegrasyonu:** Görsel çevre ve kamera verisi için.
   ```bash
   ./simulasyon_baslat.sh
   ```

3. **GCS Bağlantısı:** Yazılım varsayılan olarak `127.0.0.1:14550` portundan SITL'e bağlanır.

## 🎯 Yarışma Görevleri

### Savaşan İHA (Hava-Hava)
Otonom olarak rakip İHA'ları tespit eder ve 4 saniye boyunca kilitlenerek sanal vuruş gerçekleştirir.

### Kamikaze (Hava-Kara)
Yer hedeflerindeki QR kodu tespit eder, minimum 100m irtifadan dalış yapar, kodu okur ve yerden güvenli mesafede tekrar yükselişe geçer.

## ⚖️ Lisans
Bu proje TEKNOFEST 2026 yarışma standartlarına uygun olarak geliştirilmiştir.
