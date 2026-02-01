"""
İHA Yer Kontrol İstasyonu başlatma betiği.
"""
import sys
from PyQt5.QtWidgets import QApplication
from ground_station.ui.main_window import MainWindow

def main():
    # Uygulama oluştur
    app = QApplication(sys.argv)
    
    # Ana pencereyi oluştur ve göster
    window = MainWindow()
    window.show()
    
    try:
        # Uygulama döngüsünü başlat
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Program hatası: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    main() 