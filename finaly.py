import cv2
import numpy as np
import serial
import camara
import time

cap = cv2.VideoCapture(camara.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
#serialcomm = serial.Serial('COM7', 9600)
#serialcomm.timeout = 1
# blue
lower_colorB = [96, 109, 151]
upperColorB = [105, 255, 255]
# red
lower_colorR = [158, 100, 197]
upperColorR = [179, 255, 255]
# green
lower_colorY = [22, 71, 161]
upperColorY = [30, 255, 255]

lowerLimitsB = np.array(lower_colorB)
upperLimitsB = np.array(upperColorB)

lowerLimitsR = np.array(lower_colorR)
upperLimitsR = np.array(upperColorR)

lowerLimitsY = np.array(lower_colorY)
upperLimitsY = np.array(upperColorY)

params = cv2.SimpleBlobDetector_Params()

params.filterByColor = False
params.filterByArea = False
params.filterByInertia = False
params.filterByConvexity = False
params.filterByCircularity = False

detG = cv2.SimpleBlobDetector_create(params)
detB = cv2.SimpleBlobDetector_create(params)
detR = cv2.SimpleBlobDetector_create(params)

while True:
    success, img = cap.read()
    y = "YELLOW"
    b = "BLUE"
    r = "RED"
    img = cv2.GaussianBlur(img, (35, 35), 3)
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    imageMaskY = cv2.inRange(imgHSV, lowerLimitsY, upperLimitsY)
    imageMaskR = cv2.inRange(imgHSV, lowerLimitsR, upperLimitsR)
    imageMaskB = cv2.inRange(imgHSV, lowerLimitsB, upperLimitsB)
    resG = cv2.bitwise_and(img, img, mask=imageMaskY)
    resB = cv2.bitwise_and(img, img, mask=imageMaskB)
    resR = cv2.bitwise_and(img, img, mask=imageMaskR)
    keyPointsY = detG.detect(imageMaskY)
    keyPointsR = detR.detect(imageMaskR)
    keyPointsB = detB.detect(imageMaskB)
    cv2.drawKeypoints(img, keyPointsB, img, (0, 255, 0), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
    cv2.drawKeypoints(img, keyPointsR, img, (0, 255, 0), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
    cv2.drawKeypoints(img, keyPointsY, img, (0, 255, 0), cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
    for kb in keyPointsB:
        #serialcomm.write(b.encode())
        print(b.encode())
    for kr in keyPointsR:
        #serialcomm.write(r.encode())
        print(r.encode())
    for ky in keyPointsY:
        #serialcomm.write(y.encode())
        print(y.encode())
    cv2.imshow("image", img)

    if cv2.waitKey(100) & 0xFF == 27:  # ESC
        break
cam.release()
cv2.destroyAllWIndows()
#serialcomm.close()