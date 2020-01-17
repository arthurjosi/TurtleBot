import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec


def setlowH(newlowH):
  global lowH
  lowH = newlowH

def sethighH(newhighH):
  global highH
  highH = newhighH

def setlowS(newlowS):
  global lowS
  lowS = newlowS

def sethighS(newhighS):
  global highS
  highS = newhighS

def setlowV(newlowV):
  global lowV
  lowV = newlowV

def sethighV(newhighV):
  global highV
  highV = newhighV

lowH = 2
highH = 26
lowS = 160
highS = 255
lowV = 152
highV = 255

cap=cv2.VideoCapture(3)
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('lowH','image',lowH,179,setlowH)
cv2.createTrackbar('highH','image',highH,179,sethighH)

cv2.createTrackbar('lowS','image',lowS,255,setlowS)
cv2.createTrackbar('highS','image',highS,255,sethighS)

cv2.createTrackbar('lowV','image',lowV,255,setlowV)
cv2.createTrackbar('highV','image',highV,255,sethighV)

while True:    
    #image de référence
    #img1 = cv2.imread('image_ref.jpg')
    #blurred_frame_ref = cv2.GaussianBlur(img1, (5, 5), 0)
    #hsv_ref = cv2.cvtColor(blurred_frame_ref, cv2.COLOR_BGR2HSV)
    #_, contours1, _ = cv2.findContours(mask,cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    im = cv2.imread('image5.jpg')
    imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) 
    ret, thresh = cv2.threshold(imgray, 127, 255, 0)
    _, contours1, _= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    temp_cont = sorted(contours1, key=cv2.contourArea, reverse= True)
    new_temp_cont = temp_cont[0]
    #image à comparer
    _, frame = cap.read()
    blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([lowH,lowS,lowV])
    upper_orange = np.array([highH, highS, highV])
    mask = cv2.inRange(hsv, lower_orange, upper_orange)	
    _, contours2, _= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


    #comparaison
    for c in contours2:
    	match = cv2.matchShapes(new_temp_cont, c, 2, 0.0)
    	if match < 3.0:
            print('object detected and match = {}'.format(match))

    #cv2.imshow("Frame", frame)
    cv2.imshow("Mask", mask)
    key = cv2.waitKey(1)
    if key == 27:
        break
        
cap.release()
cv2.destroyAllWindows()
