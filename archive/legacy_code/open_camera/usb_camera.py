import cv2
cap = cv2.VideoCapture(0)  
while True:
    ret, frame = cap.read()
    if not ret:
        print("Kamera baglantisinda bir hata olustu.")

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
