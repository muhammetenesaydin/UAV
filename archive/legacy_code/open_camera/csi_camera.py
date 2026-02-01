import cv2

# CSI kamera icin GStreamer pipeline tanımlanması
video_source = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1 ! nvvidconv flip-method=2 ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink"

# GStreamer pipeline ile VideoCapture objesi olusturulması
cap = cv2.VideoCapture(video_source, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Kamera baglantısında bir hata olustu.")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Kamera baglantısında bir hata olustu.")
            break
        
        frame_height, frame_width = frame.shape[:2]
        center_x = frame_width // 2
        center_y = frame_height // 2
        box_width, box_height = 100, 100
        box_color = (0, 255, 0)
        
        cv2.rectangle(
            img=frame,
            pt1=(center_x - box_width // 2, center_y - box_height // 2),
            pt2=(center_x + box_width // 2, center_y + box_height // 2),
            color=box_color,
            thickness=2,
        )
        
        cv2.imshow("Kutu cizme", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()