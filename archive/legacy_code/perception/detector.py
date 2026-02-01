"""
Hedef tespiti ve QR kod okuma işlemlerini yapan sınıf.
"""
import cv2
import torch
import numpy as np
from utils.vision_utils import VisionUtils
from pyzbar.pyzbar import decode

class Detector:
    def __init__(self, model_path, conf_threshold=0.5):
        """
        Detector sınıfını başlatır.
        
        Args:
            model_path (str): YOLO model dosyasının yolu
            conf_threshold (float): Güven eşiği
        """
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)
        self.conf_threshold = conf_threshold
        self.vision_utils = VisionUtils()
        self.classes = ['iha']  # Tespit edilecek sınıflar
        
    def detect_target(self, image):
        """
        Görüntüde hedef tespiti yapar.
        
        Args:
            image (numpy.ndarray): İşlenecek görüntü
            
        Returns:
            list: Tespit edilen hedeflerin bilgileri
        """
        # YOLO ile tespit
        results = self.model(image)
        detections = []
        
        for *xyxy, conf, cls in results.xyxy[0]:
            if conf > self.conf_threshold:
                detections.append({
                    'bbox': [int(x) for x in xyxy],
                    'confidence': float(conf),
                    'class': int(cls)
                })
                
        return detections
        
    def detect_qr(self, image):
        """
        Görüntüde QR kod tespiti yapar.
        
        Args:
            image (numpy.ndarray): İşlenecek görüntü
            
        Returns:
            list: Tespit edilen QR kodların bilgileri
        """
        # Görüntüyü ön işle
        processed = self.vision_utils.preprocess_image(image)
        # QR kodları tespit et
        qr_codes = self.vision_utils.detect_qr_code(processed)
        return qr_codes
    
    def enhanced_qr_detection(self, image, apply_filters=True, multiple_attempts=True):
        """
        Gelişmiş QR kod tespiti yapar. Farklı ön işleme teknikleri kullanarak
        QR kod okuma başarısını artırır.
        
        Args:
            image (numpy.ndarray): İşlenecek görüntü
            apply_filters (bool): Görüntü filtreleri uygulansın mı?
            multiple_attempts (bool): Birden fazla teknikle deneme yapılsın mı?
            
        Returns:
            dict: QR kod bilgileri ve tespit parametreleri
        """
        if image is None or image.size == 0:
            return None
            
        # Orijinal görüntüde deneme
        original_qr = self.vision_utils.detect_qr_code(image)
        if original_qr and len(original_qr) > 0:
            return {
                'qr_data': original_qr,
                'method': 'original',
                'success': True
            }
            
        if not multiple_attempts:
            return {'success': False}
            
        # Farklı ön işleme teknikleri
        techniques = []
        results = []
        
        # 1. Klasik gri tonlama ve bulanıklaştırma
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        techniques.append(('gray_blur', blurred))
        
        if apply_filters:
            # 2. Kontrast artırma
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            techniques.append(('contrast_enhanced', enhanced))
            
            # 3. Keskinleştirme filtresi
            sharpening_kernel = np.array([[-1, -1, -1], 
                                        [-1, 9, -1], 
                                        [-1, -1, -1]])
            sharpened = cv2.filter2D(gray, -1, sharpening_kernel)
            techniques.append(('sharpened', sharpened))
            
            # 4. Adaptif eşikleme
            thresh = cv2.adaptiveThreshold(gray, 255, 
                                         cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                         cv2.THRESH_BINARY, 11, 2)
            techniques.append(('adaptive_thresh', thresh))
            
            # 5. Kenar tespiti (Canny)
            edges = cv2.Canny(gray, 100, 200)
            dilated_edges = cv2.dilate(edges, np.ones((3, 3), np.uint8), iterations=1)
            techniques.append(('edges', dilated_edges))
        
        # Tüm teknikleri dene
        for name, processed_img in techniques:
            qr_results = self.vision_utils.detect_qr_code(processed_img)
            if qr_results and len(qr_results) > 0:
                results.append({
                    'qr_data': qr_results,
                    'method': name,
                    'success': True
                })
        
        # Sonuçları değerlendir
        if results:
            # En güvenilir sonucu seç (ilk bulunanı)
            return results[0]
        
        # Hiçbir yöntem başarılı olmadı
        return {
            'success': False,
            'methods_tried': len(techniques) + 1  # +1 orijinal görüntü
        }
    
    def track_and_read_qr(self, frame, previous_qr_info=None, roi_expand=20):
        """
        QR kodları izler ve okur. Önceki karelerde tespit edilen QR kodların
        bulunduğu bölgelere odaklanarak performansı artırır.
        
        Args:
            frame (numpy.ndarray): Kamera görüntüsü
            previous_qr_info (dict): Önceki karede tespit edilen QR kod bilgisi
            roi_expand (int): İlgi bölgesini genişletme miktarı (piksel)
            
        Returns:
            dict: QR kod bilgileri ve takip durumu
        """
        height, width = frame.shape[:2]
        result = {
            'success': False,
            'qr_data': None,
            'tracking': False
        }
        
        # Önceki tespite göre ROI bölgesi oluştur
        if previous_qr_info and 'rect' in previous_qr_info:
            prev_rect = previous_qr_info['rect']
            x, y, w, h = prev_rect
            
            # ROI sınırlarını hesapla
            x1 = max(0, x - roi_expand)
            y1 = max(0, y - roi_expand)
            x2 = min(width, x + w + roi_expand)
            y2 = min(height, y + h + roi_expand)
            
            # ROI bölgesini kes
            roi = frame[y1:y2, x1:x2]
            
            # ROI'de QR tespiti
            qr_results = self.enhanced_qr_detection(roi)
            
            if qr_results and qr_results.get('success', False):
                # Koordinatları orijinal görüntüye göre ayarla
                for qr in qr_results.get('qr_data', []):
                    if 'rect' in qr:
                        rect = qr['rect']
                        qr['rect'] = (rect[0] + x1, rect[1] + y1, rect[2], rect[3])
                    if 'points' in qr:
                        points = qr['points']
                        qr['points'] = [(p[0] + x1, p[1] + y1) for p in points]
                
                result = {
                    'success': True,
                    'qr_data': qr_results.get('qr_data', []),
                    'tracking': True,
                    'method': qr_results.get('method', 'roi_tracking')
                }
                return result
        
        # ROI'de tespit yapılamazsa veya önceki bilgi yoksa tüm görüntüyü tara
        qr_results = self.enhanced_qr_detection(frame)
        
        if qr_results and qr_results.get('success', False):
            result = {
                'success': True,
                'qr_data': qr_results.get('qr_data', []),
                'tracking': False,
                'method': qr_results.get('method', 'full_scan')
            }
            
        return result
    
    def advanced_qr_detection_using_formulas(self, frame, camera_params=None):
        """
        Kamera formüllerini kullanarak QR kodu tespit etmek için gelişmiş bir metot.
        Tespit edilen QR kodlarını mesafeye göre sınıflandırır ve ek bilgiler ekler.
        
        Args:
            frame (numpy.ndarray): Kamera görüntüsü
            camera_params (dict): Kamera parametreleri (opsiyonel)
                - focal_length: Odak uzaklığı (mm)
                - fov: Görüş açısı (derece)
                - sensor_width: Sensör genişliği (mm)
                - qr_size: QR kod boyutu (metre)
                
        Returns:
            dict: QR tespiti sonuçları ve ek bilgiler
        """
        if frame is None or frame.size == 0:
            return {'success': False, 'reason': 'Invalid frame'}
            
        # Varsayılan kamera parametreleri
        if camera_params is None:
            camera_params = {
                'focal_length': 3.6,     # mm
                'fov': 60.0,             # derece
                'sensor_width': 4.8,     # mm (1/3" sensör)
                'qr_size': 0.1           # 10 cm QR kodu
            }
            
        # Görüntü boyutları
        height, width = frame.shape[:2]
        
        # Formüllere dayalı hesaplamalar
        focal_length = camera_params['focal_length']
        fov = camera_params['fov']
        sensor_width = camera_params['sensor_width']
        qr_size = camera_params['qr_size']
        
        # Minimum QR kod piksel genişliği
        min_pixel_width = 30  # 30 piksel altında QR kodunu tespit etmek zor olabilir
        
        # 1. Minimum tespit mesafesi
        # d_min = (qr_width * focal_length) / min_pixel_width
        min_distance = (qr_size * focal_length) / min_pixel_width
        
        # 2. Maksimum tespit mesafesi
        # d = (2 * tan(θ/2) * min_pixel_width * qr_width) / resolution_width
        fov_rad = np.radians(fov)
        max_distance = (2 * np.tan(fov_rad / 2) * min_pixel_width * qr_size) / width
        
        # 3. Yerden görülen alan (footprint) - 1 metre mesafede
        # F = 2 * h * tan(θ/2)
        one_meter_footprint = 2 * 1.0 * np.tan(fov_rad / 2)
        
        # 4. Piksel/metre oranı - 1 metre mesafede
        # Piksel/metre = resolution_width / footprint
        pixel_per_meter = width / one_meter_footprint
        
        # QR kod tespiti
        enhanced_qr = self.enhanced_qr_detection(frame)
        
        # Sonuç verisini oluştur
        result = {
            'success': enhanced_qr.get('success', False),
            'min_detection_distance': min_distance,
            'max_detection_distance': max_distance,
            'pixel_per_meter': pixel_per_meter,
            'footprint_at_1m': one_meter_footprint,
            'camera_params': camera_params
        }
        
        # Eğer QR kodu tespit edildiyse, ek bilgiler ekle
        if result['success'] and enhanced_qr.get('qr_data'):
            qr_list = enhanced_qr['qr_data']
            processed_qr_list = []
            
            for qr in qr_list:
                # QR kod boyutunu kullanarak mesafe tahmini
                if 'rect' in qr:
                    rect = qr['rect']
                    qr_width_px = rect[2]  # Piksel cinsinden genişlik
                    
                    # Gerçek dünyada QR koduna olan mesafeyi tahmin et
                    # w_screen = (focal_length * qr_width) / distance
                    # distance = (focal_length * qr_width) / w_screen
                    estimated_distance = (focal_length * qr_size) / qr_width_px
                    
                    # QR kodun görüntüdeki oranı (FOV'un yüzdesi)
                    qr_view_ratio = qr_width_px / width
                    
                    # Dinamik eşik değeri
                    # w_threshold = (safe_distance * qr_width) / focal_length
                    safe_distance = min_distance * 1.2
                    dynamic_threshold = (safe_distance * qr_size) / focal_length
                    
                    # QR genişliği minimum değerden büyükse ve makul bir mesafedeyse
                    is_reliable = qr_width_px >= min_pixel_width and estimated_distance < max_distance * 1.5
                    
                    # Merkez koordinatları
                    center_x = rect[0] + rect[2] / 2
                    center_y = rect[1] + rect[3] / 2
                    
                    # Normalize edilmiş merkez (-1 ile 1 arası)
                    norm_center_x = (center_x - width/2) / (width/2)  
                    norm_center_y = (center_y - height/2) / (height/2)
                    
                    # QR bilgilerini genişlet
                    qr.update({
                        'estimated_distance': estimated_distance,
                        'qr_size_px': qr_width_px,
                        'qr_view_ratio': qr_view_ratio,
                        'is_reliable': is_reliable,
                        'dynamic_threshold': dynamic_threshold,
                        'center': (center_x, center_y),
                        'normalized_center': (norm_center_x, norm_center_y)
                    })
                    
                    processed_qr_list.append(qr)
            
            result['qr_data'] = processed_qr_list
            result['qr_count'] = len(processed_qr_list)
            result['detection_method'] = enhanced_qr.get('method', 'unknown')
            
            # En güvenilir QR kodu seç
            if processed_qr_list:
                # Güvenilir QR kodlarını filtrele
                reliable_qr = [qr for qr in processed_qr_list if qr.get('is_reliable', False)]
                
                if reliable_qr:
                    # En büyük QR kodunu seç (muhtemelen en yakında olan)
                    best_qr = max(reliable_qr, key=lambda x: x.get('qr_size_px', 0))
                    result['best_qr'] = best_qr
        
        return result
        
    def is_target_locked(self, detections, min_duration=4):
        """
        Hedefin kilitlenip kilitlenmediğini kontrol eder.
        
        Args:
            detections (list): Tespit edilen hedefler
            min_duration (int): Minimum kilitlenme süresi (saniye)
            
        Returns:
            bool: Hedef kilitlendi mi?
        """
        if not detections:
            return False
            
        # Hedef görüntüde varsa ve minimum süre geçtiyse True döndür
        return len(detections) > 0
        
    def detect_iha(self, frame, target_id=None):
        """
        İHA tespiti yapar.
        
        Args:
            frame: Kamera görüntüsü
            target_id: Hedef İHA ID'si (opsiyonel)
            
        Returns:
            dict: Tespit edilen İHA bilgileri
        """
        # Görüntüyü YOLO için hazırla
        blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.model.setInput(blob)
        
        # Tespit yap
        layer_names = self.model.getLayerNames()
        output_layers = [layer_names[i-1] for i in self.model.getUnconnectedOutLayers()]
        outputs = self.model.forward(output_layers)
        
        # Tespit sonuçlarını işle
        ihas = []
        height, width = frame.shape[:2]
        
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                
                if confidence > 0.5 and self.classes[class_id] == 'iha':
                    # Sınırlayıcı kutu koordinatlarını hesapla
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    
                    # Sol üst köşe koordinatları
                    x = int(center_x - w/2)
                    y = int(center_y - h/2)
                    
                    # İHA bilgilerini kaydet
                    iha_info = {
                        'id': target_id if target_id else len(ihas),
                        'position': (center_x, center_y),
                        'bbox': (x, y, w, h),
                        'confidence': float(confidence)
                    }
                    
                    ihas.append(iha_info)
        
        # Hedef ID belirtilmişse, o İHA'yı döndür
        if target_id is not None:
            for iha in ihas:
                if iha['id'] == target_id:
                    return iha
            return None
            
        return ihas if ihas else None
        
    def track_iha(self, frame, target_iha, tracker_type='CSRT'):
        """
        İHA takibi yapar.
        
        Args:
            frame: Kamera görüntüsü
            target_iha: Takip edilecek İHA bilgileri
            tracker_type: Takip algoritması tipi
            
        Returns:
            tuple: Takip durumu ve yeni sınırlayıcı kutu
        """
        # Takipçiyi başlat
        if not hasattr(self, 'tracker'):
            bbox = target_iha['bbox']
            if tracker_type == 'CSRT':
                self.tracker = cv2.TrackerCSRT_create()
            else:
                self.tracker = cv2.TrackerKCF_create()
            self.tracker.init(frame, bbox)
            
        # Takip et
        success, bbox = self.tracker.update(frame)
        
        if success:
            # Yeni pozisyonu hesapla
            x, y, w, h = [int(v) for v in bbox]
            center_x = x + w/2
            center_y = y + h/2
            
            return True, (center_x, center_y)
            
        return False, None 