import cv2
import camara 
print(cv2.__version__)


cam= cv2.VideoCapture(camara.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

while True:
    success, img = cam.read()
    cv2.imshow("picam",img)
    if cv2.waitKey(1) & 0xFF == 27:
        break


cam.release()
cv2.destroyAllWIndows()