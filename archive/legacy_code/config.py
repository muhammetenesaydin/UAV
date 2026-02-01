mission_config = {
    # Hedef noktasının GPS koordinatları. Uçak bu noktaya kamikaze dalışı yapacak.
    "hedef": {
        "lat": -35.36318300,       # Enlem (latitude) değeri
        "lon": 149.16528000,      # Boylam (longitude) değeri
        "aciklama": "QR kod hedefi, kamikaze dalış yapılacak koordinatlar"
    
    },

    # Uçuşa ilişkin genel parametreler
    "ucus_parametreleri": {
        "target_altitude": 120,     # Kalkış sonrası ulaşılacak güvenli seyir irtifası (metre)
        "approach_threshold": 25,  # Hedefe yaklaşma eşiği; bu mesafeye gelince dalışa hazırlık başlar (metre)
        "camera_url": "http://0.0.0.0:8080/stream?topic=/cessna/image_raw",  # QR kod taraması için kullanılacak kamera akışı URL'si
        "aciklama": "Genel uçuş parametreleri"
    },

    # Dalış (kamikaze) sırasında uçuş kontrolcüsüne yüklenecek TECS ve diğer parametreler
    "dalis_parametreleri": {
        "TECS_SINK_MAX": 15.0,     # Maksimum izin verilen iniş hızı (m/s) - Yavaşlatıldı daha uzun dalış için
        "TECS_SINK_MIN": 8.0,      # Minimum iniş hızı (m/s) - Yavaşlatıldı daha uzun dalış için
        "LIM_PITCH_MAX": 4500,     # Maksimum burun yukarı pitch limiti (milidegree)
        "LIM_PITCH_MIN": -30000,    # Maksimum burun aşağı pitch limiti (milidegree) - Daha dik dalış için
        "LAND_PITCH_CD": -25000,    # İnişte kullanılacak pitch değeri (milidegree) - Daha dik dalış için
        "TECS_TIME_CONST": 5.0,    # TECS kontrolcüsünün tepki zaman sabiti (saniye) - Artırıldı daha yavaş tepki için
        "TECS_SPDWEIGHT": 0.7,     # Hız ve yükseklik kontrol öncelik ağırlığı - Yükseklik kontrolü ağırlıklı
        "THR_MAX": 100.0,          # Motorun maksimum throttle (gaz) limiti (%)
        "WP_RADIUS": 15.0,         # Waypoint kabul yarıçapı (metre) - Artırıldı daha erken geçiş için
        "NAVL1_PERIOD": 20.0,      # NAV L1 kontrol periyodu (saniye) - Artırıldı daha yumuşak yörünge için
        "aciklama": "Kamikaze dalış sırasında kullanılacak parametreler (TECS ve diğer uçuş kontrolcü parametreleri)"
    },

    # Görev davranışına ilişkin parametreler
    "gorev_parametreleri": {
        "waypoint_kabul_yaricapi": {
            "normal": 5,            # Seyir aşamasında waypoint kabul yarıçapı (metre)
            "dalis": 2,             # Dalış aşamasında waypoint kabul yarıçapı (metre)
            "aciklama": "Waypoint kabul yarıçapları (metre cinsinden)"
        },
        "yaklasim_mesafeleri": {
            "qr_scan_baslat": 150,   # Hedefe 150m yaklaşınca QR taramayı başlat (daha erken başla)
            "dalis_baslat": 120,     # Hedefe 120m yaklaşınca dalışa geçiş (daha erken dalış başlat)
            "zorla_dalis": 80,      # Zorunlu dalış başlatma mesafesi (emergency)
            "aciklama": "Hedefe yaklaşma mesafeleri (metre cinsinden)"
        },
        "dalis_hedefi": {
            "irtifa": 1,            # Dalış tamamlandığında ulaşılacak hedef irtifa (metre)
            "aciklama": "Dalışta ulaşılacak hedef irtifa (metre cinsinden)"
        },
        "dalis_suresi": {
            "min_sure": 8,          # Minimum dalış süresi (saniye)
            "ekstra_bekleme": 3,    # İstenen irtifaya ulaştıktan sonra ek bekleme süresi (saniye)
            "max_sure": 20,         # Maksimum dalış süresi (saniye)
            "rtl_gecikme": 5,       # Dalış sonrası RTL'e geçmeden önce bekleme süresi (saniye)
            "aciklama": "Dalış süresi kontrol parametreleri (saniye cinsinden)"
        },
        "kurtarma": {
            "pitch_up": 1100,       # Burun yukarı PWM değeri (1000-1100 arası maksimum yukarı)
            "tekrar_sayisi": 10,    # Burun yukarı komut tekrar sayısı
            "throttle": 1900,       # Kurtarma sırasında motor gücü (tam güç)
            "beklemeler": {
                "pitch_up": 0.1,    # Burun yukarı komutları arası bekleme süresi (saniye)
                "tam_gaz": 1.0,     # Tam gaz ve burun yukarı pozisyonunda bekleme süresi (saniye)
                "qr_tolerans": 0.5, # QR tespit edildikten sonra kurtarma öncesi bekleme süresi (saniye)
            },
            "normal_deger": {
                "trim": 0,          # Normal TRIM_PITCH_CD değeri
                "pitch_min": -4500  # Normal LIM_PITCH_MIN değeri
            },
            "aciklama": "Dalış sonrası kurtarma ve burun yukarı dikme parametreleri"
        },
        "guvenli_irtifa": 80,      # Dalış öncesi ve sonrası güvenli yükseklik (metre)
        "aciklama": "Göreve ilişkin parametre değerleri"
    },

    # Kalkış aşamasında kullanılacak RC override değerleri
    "kalkis_parametreleri": {
        "throttle_degerleri": [1700, 1800, 1900],  # Kalkış sırasında adım adım throttle PWM değerleri
        "pitch_up": 1300,        # Kalkışta burun yukarı pitch PWM ayarı
        "throttle_kalkis": 1900, # Kalkış anında tam gaz PWM değeri
        "yunus_hareketi": {
            "sure": 15,         # Yunus hareketi toplam süresi (saniye)
            "yukari_sure": 3,  # Burun yukarı tutma süresi (saniye)
            "asagi_sure": 0.8,   # Burun aşağı tutma süresi (saniye)
            "pitch_down": 1600,  # Burun aşağı pitch PWM değeri
            "pitch_up": 1200,    # Burun yukarı pitch PWM değeri
            "aciklama": "Yunus hareketi özellikleri"
        },
        "aciklama": "Kalkış aşamasında kullanılacak RC override değerleri"
    },

    # QR kod tarama parametreleri
    "qr_tarama": {
        "tarama_siklik": {
            "normal": 1.5,           # Seyir modunda her 1.5 saniyede bir tarama denemesi
            "dalis": 0.5,            # Dalış modunda her 0.5 saniyede bir tarama denemesi
            "aciklama": "QR tarama sıklığı (saniye cinsinden)"
        },
        "tarama_suresi": {
            "normal": 1,             # Her tarama için maksimum süre (s)
            "dalis": 2,              # Dalış esnasında tarama için maksimum süre (s)
            "aciklama": "Her tarama için maksimum süre (saniye cinsinden)"
        },
        "alternatif_kamera_urls": [
            "http://localhost:8080/stream?topic=/cessna/image_raw",
            "http://127.0.0.1:8080/stream?topic=/cessna/camera/image",
            "http://127.0.0.1:8080/?action=stream",
            0                       # Geçersiz URL referans numarası, atlama için
        ],
        "aciklama": "QR taramaya ilişkin parametreler"
    },

    # Burun aşağı çevirme (dolaylı dalış) için RC override ayarları
    "pitch_override": {
        "normal": 2000,           # Normal dalış başlangıcı için burun aşağı PWM değeri (daha aşağı)
        "agresif": 2047,          # Agresif dalış için burun aşağı PWM değeri (maksimum)
        "tekrarlama": {
            "normal": 8,           # Normal PWM komutu tekrarlama sayısı
            "agresif": 12         # Agresif PWM komutu tekrarlama sayısı
        },
        "aciklama": "Burnu aşağı çevirmek için RC override değerleri ve tekrarlama sayıları"
    }
}
