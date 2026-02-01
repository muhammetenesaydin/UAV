# UDP Görüntü Aktarım Sistemi

Bu proje, UDP protokolü kullanarak kamera görüntüsünü gerçek zamanlı olarak ağ üzerinden aktaran basit bir sistem içermektedir.

## Dosyalar

- `sender.py` - Kamera görüntüsünü UDP üzerinden gönderen program
- `receiver.py` - UDP üzerinden görüntü alan program
- `requirements.txt` - Gerekli Python paketleri

## Kurulum

1. Gerekli paketleri yükleyin:
```bash
pip install -r requirements.txt
```

## Kullanım

### 1. Sender (Gönderici) Çalıştırma

```bash
python sender.py
```

Program başladığında:
- Kameraınız açılacak
- IP adresi ve port numarası ekranda görünecek
- Bu bilgileri not alın (receiver'da kullanacaksınız)
- Çıkmak için 'q' tuşuna basın

### 2. Receiver (Alıcı) Çalıştırma

```bash
python receiver.py
```

Program başladığında:
- Size IP adresi sorulacak (sender'dan aldığınız IP)
- Size port numarası sorulacak (sender'dan aldığınız port)
- Görüntü almaya başlayacak
- Çıkmak için 'q' tuşuna basın

## Özellikler

- **Sender:**
  - Otomatik IP adresi tespiti
  - Kamera görüntüsü üzerinde IP/Port gösterimi
  - 640x480 çözünürlük, 30 FPS
  - JPEG sıkıştırma ile veri tasarrufu

- **Receiver:**
  - Kullanıcı dostu IP/Port girişi
  - Gerçek zamanlı görüntü alımı
  - Bağlantı durumu gösterimi

## Teknik Detaylar

- **Protocol:** UDP
- **Port:** 9999 (varsayılan)
- **Görüntü Formatı:** JPEG
- **Çözünürlük:** 640x480
- **Sıkıştırma:** %80 JPEG kalite

## Notlar

- Kamera izinlerinizin açık olduğundan emin olun
- Aynı ağda olduğunuzdan emin olun
- Firewall ayarlarınızı kontrol edin 