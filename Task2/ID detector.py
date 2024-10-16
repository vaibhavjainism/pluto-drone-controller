import cv2
from cv2 import aruco
import numpy as np

def findArucoMarkers(img, draw=True):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # because it has only one channel
    key = getattr(aruco, f'DICT_4X4_250') # max marker detected = 250
    arucoDict = aruco.getPredefinedDictionary(key)
    arucoParam = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(arucoDict, arucoParam)
    corners, ids, rejected = detector.detectMarkers(gray)

    if len(corners) > 0: # marker detected in frame
        print(ids)
        ids = ids.flatten() #drops one dimension
        print(ids)

        for (markerCounter, markerID) in zip(corners, ids):
            corners = markerCounter.reshape((4,2))
            (tl, tr, br, bl) = corners

            tr = (int(tr[0]), int(tr[1]))
            tl = (int(tl[0]), int(tl[1]))
            bl = (int(bl[0]), int(bl[1]))
            br = (int(br[0]), int(br[1]))

            cv2.putText(img, str(markerID), tl,cv2.FONT_HERSHEY_COMPLEX, 1, (0,255,0), 2, cv2.LINE_AA)
            cv2.rectangle(img, tl, br, (255,0,0), 5)

            cv2.line(img, tl, tr, (99,250,205), 2)
            cv2.line(img, tr, br, (99,250,205), 2)
            cv2.line(img, br, bl, (99,250,205), 2)
            cv2.line(img, bl, tl, (99,250,205), 2)

            cv2.line(img, tl, tr, (255,0,0), 2)
            cv2.line(img, tr, br, (0,255,0), 2)
            cv2.line(img, br, bl, (0,0,255), 2)
            cv2.line(img, bl, tl, (0,0,0), 2)

cap = cv2.VideoCapture(0)

while True:
    check, frame = cap.read()

    if check:
        findArucoMarkers(frame)
        cv2.imshow("image", frame)

    if cv2.waitKey(10) & 0xFF == ord('q'): 
        cap.release() 
        break
cap.release()
cv2.destroyAllWindows()
