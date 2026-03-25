import cv2
import pytesseract
import numpy as np

def check_victim(side):
    if side == "right":
        x = 0
    else: 
        x = 1 
    cap = cv2.VideoCapture(x)
    config = r"--oem 3 --psm 10 -c tessedit_char_whitelist=HSU"
    detected_text = None
    while True:
        ret, frame = cap.read()
        if not ret:
            print("no frame")
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, th = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY_INV)
        white_pixels = np.sum(th > 127)
        total_pixels = th.shape[0] * th.shape[1]
        text_ratio = white_pixels / total_pixels
        if text_ratio > 0.01:
            text = pytesseract.image_to_string(th, config=config).strip()       

            if text:
                print("Detected:", text)
                detected_text = text
                break
    cap.release()
    cv2.destroyAllWindows()
    return detected_text
