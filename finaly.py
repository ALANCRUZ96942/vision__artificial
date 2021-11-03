import cv2
import numpy as np
import camara
import RPi.GPIO as gpio

cap = cv2.VideoCapture(camara.gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

# blue
lower_colorB = [96, 33, 151]
upperColorB = [105, 255, 255]
# red
lower_colorR = [90, 90, 160]
upperColorR = [179, 120, 255]
# yellow
lower_colorY = [20, 71, 161]
upperColorY = [90, 255, 255]

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

gpio.setwarnings(False)
gpio.setmode(gpio.BOARD)
gpio.setup(7,gpio.OUT)
gpio.setup(11,gpio.OUT)
gpio.setup(13,gpio.OUT)
gpio.setup(15,gpio.OUT)
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
        print(b.encode())
        gpio.output(7,True)
        gpio.output(11,False)
        gpio.output(13,False)
        gpio.output(15,False)

    for kr in keyPointsR:
        print(r.encode())
        gpio.output(7,False)
        gpio.output(11,True)
        gpio.output(13,False)
        gpio.output(15,True)

    for ky in keyPointsY:
        print(y.encode())
        gpio.output(7,False)
        gpio.output(11,False)
        gpio.output(13,True)
        gpio.output(15,True)
    
    cv2.imshow("image", img)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC
        gpio.output(7,False)
        gpio.output(11,False)
        gpio.output(13,False)
        gpio.output(15,False)
        break
cam.release()
cv2.destroyAllWIndows()
#serialcomm.close()
