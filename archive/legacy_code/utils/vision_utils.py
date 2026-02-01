"""
Görüntü işleme için yardımcı fonksiyonlar içeren modül.
"""
import cv2
import numpy as np
from pyzbar.pyzbar import decode

class VisionUtils:
    @staticmethod
    def preprocess_image(image):
        """
        Görüntüyü ön işlemeden geçirir.
        
        Args:
            image (numpy.ndarray): İşlenecek görüntü
            
        Returns:
            numpy.ndarray: Ön işlenmiş görüntü
        """
        # Görüntüyü gri tonlamaya çevir
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Gürültü azaltma
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        return blurred
        
    @staticmethod
    def detect_qr_code(image):
        """
        Görüntüdeki QR kodları tespit eder.
        
        Args:
            image (numpy.ndarray): İşlenecek görüntü
            
        Returns:
            list: Tespit edilen QR kodların bilgileri
        """
        decoded_objects = decode(image)
        results = []
        
        for obj in decoded_objects:
            results.append({
                'data': obj.data.decode('utf-8'),
                'points': obj.polygon,
                'rect': obj.rect
            })
            
        return results
        
    @staticmethod
    def calculate_camera_fov(focal_length, sensor_width):
        """
        Kamera görüş açısını hesaplar (FOV = Field of View).
        
        Args:
            focal_length (float): Odak uzaklığı (mm)
            sensor_width (float): Sensör genişliği (mm)
            
        Returns:
            float: Görüş açısı (derece)
        """
        # FOV = 2 * arctan(sensor_width / (2 * focal_length))
        fov_rad = 2 * np.arctan(sensor_width / (2 * focal_length))
        fov_deg = np.degrees(fov_rad)
        return fov_deg
    
    @staticmethod
    def calculate_footprint(height, fov):
        """
        Belirli bir yükseklikten görülebilen alan genişliğini hesaplar.
        
        Args:
            height (float): Uçuş yüksekliği (m)
            fov (float): Görüş açısı (derece)
            
        Returns:
            float: Yerden görülen alan genişliği (m)
        """
        # F = 2 * h * tan(θ/2)
        fov_rad = np.radians(fov)
        footprint = 2 * height * np.tan(fov_rad / 2)
        return footprint
    
    @staticmethod
    def calculate_qr_pixel_size(distance, qr_width, focal_length):
        """
        QR kodunun ekrandaki piksel cinsinden büyüklüğünü hesaplar.
        
        Args:
            distance (float): QR koduna olan mesafe (m)
            qr_width (float): QR kodunun gerçek genişliği (m)
            focal_length (float): Odak uzaklığı (mm)
            
        Returns:
            float: QR kodunun piksel cinsinden genişliği
        """
        # w = (qr_width * focal_length) / distance
        pixel_size = (qr_width * focal_length) / distance
        return pixel_size
    
    @staticmethod
    def calculate_min_detection_distance(min_pixel_width, qr_width, focal_length):
        """
        QR kodunun tespit edilebileceği minimum mesafeyi hesaplar.
        
        Args:
            min_pixel_width (float): Minimum piksel genişliği 
            qr_width (float): QR kodunun gerçek genişliği (m)
            focal_length (float): Odak uzaklığı (mm)
            
        Returns:
            float: Minimum tespit mesafesi (m)
        """
        # d_min = (qr_width * focal_length) / min_pixel_width
        min_distance = (qr_width * focal_length) / min_pixel_width
        return min_distance
    
    @staticmethod
    def calculate_max_detection_distance(min_pixel_width, qr_width, fov, resolution_width):
        """
        QR kodunun okunabileceği maksimum mesafeyi hesaplar.
        
        Args:
            min_pixel_width (float): Minimum piksel genişliği
            qr_width (float): QR kodunun gerçek genişliği (m)
            fov (float): Görüş açısı (derece)
            resolution_width (int): Kamera çözünürlük genişliği (piksel)
            
        Returns:
            float: Maksimum tespit mesafesi (m)
        """
        # d = (2 * tan(θ/2) * min_pixel_width * qr_width) / resolution_width
        fov_rad = np.radians(fov)
        max_distance = (2 * np.tan(fov_rad / 2) * min_pixel_width * qr_width) / resolution_width
        return max_distance
    
    @staticmethod
    def calculate_dive_start_distance(min_distance):
        """
        Dalış başlangıç noktasının mesafesini hesaplar.
        
        Args:
            min_distance (float): Minimum tespit mesafesi (m)
            
        Returns:
            float: Dalış başlangıç mesafesi (m)
        """
        # d_start = 3 * d_min
        start_distance = 3 * min_distance
        return start_distance
    
    @staticmethod
    def calculate_altitude_from_footprint(desired_footprint, fov):
        """
        İstenilen footprint'e göre gereken irtifayı hesaplar.
        
        Args:
            desired_footprint (float): İstenilen görüş alanı genişliği (m)
            fov (float): Kamera görüş açısı (derece)
            
        Returns:
            float: Gereken irtifa (m)
        """
        # h = F / (2 * tan(θ/2))
        fov_rad = np.radians(fov)
        height = desired_footprint / (2 * np.tan(fov_rad / 2))
        return height
    
    @staticmethod
    def calculate_pixel_per_meter(footprint, resolution_width):
        """
        Piksel/metre oranını hesaplar.
        
        Args:
            footprint (float): Görüntülenen alanın genişliği (m)
            resolution_width (int): Kamera çözünürlük genişliği (piksel)
            
        Returns:
            float: Piksel/metre oranı
        """
        # Piksel/metre = resolution_width / footprint
        pixel_per_meter = resolution_width / footprint
        return pixel_per_meter
    
    @staticmethod
    def calculate_qr_screen_width(focal_length, qr_width, distance):
        """
        QR kodunun ekrandaki piksel genişliğini hesaplar.
        
        Args:
            focal_length (float): Odak uzaklığı (mm)
            qr_width (float): QR kodunun gerçek genişliği (m)
            distance (float): QR koduna olan mesafe (m)
            
        Returns:
            float: QR kodunun ekrandaki piksel genişliği
        """
        # w_screen = (focal_length * qr_width) / distance
        screen_width = (focal_length * qr_width) / distance
        return screen_width
    
    @staticmethod
    def calculate_dynamic_threshold(safe_distance, qr_width, focal_length):
        """
        Dinamik eşik değerini hesaplar.
        
        Args:
            safe_distance (float): Güvenli tespit mesafesi (m)
            qr_width (float): QR kodunun gerçek genişliği (m)
            focal_length (float): Odak uzaklığı (mm)
            
        Returns:
            float: Dinamik eşik değeri (piksel)
        """
        # w_threshold = (safe_distance * qr_width) / focal_length
        threshold = (safe_distance * qr_width) / focal_length
        return threshold
    
    @staticmethod
    def calculate_optimal_detection_parameters(qr_size=0.1, camera_fov=60, 
                                            focal_length=3.6, min_pixel_size=30, 
                                            resolution=(1920, 1080)):
        """
        QR kod tespiti için optimum parametreleri hesaplar.
        
        Args:
            qr_size (float): QR kodunun gerçek boyutu (m)
            camera_fov (float): Kamera görüş açısı (derece)
            focal_length (float): Odak uzaklığı (mm)
            min_pixel_size (int): Minimum tespit edilebilir piksel boyutu
            resolution (tuple): Kamera çözünürlüğü (genişlik, yükseklik)
            
        Returns:
            dict: QR tespiti için optimum parametreler
        """
        resolution_width, resolution_height = resolution
        
        # Sensör genişliğini hesapla (geri mühendislik ile)
        sensor_width = 2 * focal_length * np.tan(np.radians(camera_fov/2))
        
        # Minimum tespit mesafesi
        min_distance = VisionUtils.calculate_min_detection_distance(
            min_pixel_size, qr_size, focal_length)
        
        # Maksimum tespit mesafesi
        max_distance = VisionUtils.calculate_max_detection_distance(
            min_pixel_size, qr_size, camera_fov, resolution_width)
        
        # Dalış başlangıç mesafesi
        dive_start_distance = VisionUtils.calculate_dive_start_distance(min_distance)
        
        # İdeal görüntüleme yüksekliği için footprint hesapla
        # QR kodun FOV'un %25'ini kapladığı durumda
        ideal_footprint = qr_size / 0.25
        ideal_altitude = VisionUtils.calculate_altitude_from_footprint(
            ideal_footprint, camera_fov)
        
        # QR kodun ekranda 100 piksel genişliğinde görünmesi için mesafe
        optimal_distance = (focal_length * qr_size * resolution_width) / (100 * sensor_width)
        
        return {
            'min_detection_distance': min_distance,
            'max_detection_distance': max_distance,
            'dive_start_distance': dive_start_distance,
            'ideal_altitude': ideal_altitude,
            'optimal_distance': optimal_distance,
            'sensor_width_mm': sensor_width,
            'pixel_per_meter_at_10m': resolution_width / VisionUtils.calculate_footprint(10, camera_fov)
        }
        
    @staticmethod
    def draw_detections(image, detections):
        """
        Tespit edilen nesneleri görüntü üzerine çizer.
        
        Args:
            image (numpy.ndarray): Orijinal görüntü
            detections (list): Tespit edilen nesneler
            
        Returns:
            numpy.ndarray: İşaretlenmiş görüntü
        """
        output = image.copy()
        
        for det in detections:
            # Sınırlayıcı kutu çiz
            x1, y1, x2, y2 = det['bbox']
            cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Sınıf ve güven skoru yaz
            label = f"{det['class']}: {det['confidence']:.2f}"
            cv2.putText(output, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
        return output 