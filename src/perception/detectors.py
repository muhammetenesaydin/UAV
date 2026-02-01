import cv2
import numpy as np
import torch
from src.core.interfaces import IDetector

class YOLODetector(IDetector):
    def __init__(self, model_path, confidence=0.5):
        # NOT: Gerçek projede torch.hub.load veya ultralytics kullanılabilir
        # Burada örnek bir yapı kuruluyor
        self.model_path = model_path
        self.confidence = confidence
        self.model = None # Yükleme mantığı buraya gelecek

    def detect(self, frame):
        # Tespit mantığı
        results = []
        # results.append({'bbox': [x, y, w, h], 'conf': 0.9, 'class': 'iha'})
        return results

class QRDetector(IDetector):
    def __init__(self):
        from pyzbar.pyzbar import decode
        self.decode = decode

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded_objects = self.decode(gray)
        results = []
        for obj in decoded_objects:
            results.append({
                'data': obj.data.decode('utf-8'),
                'rect': obj.rect,
                'polygon': obj.polygon
            })
        return results
