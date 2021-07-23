import cv2 as cv
import numpy as np


image = cv.imread("/home/keegan/Pictures/TerrainExample1.png")
print(image)
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
ret, thresh = cv.threshold(gray, 30, 255,cv.THRESH_BINARY_INV)
contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

max_contour = max(contours, key=cv.contourArea)
area = cv.contourArea(max_contour)
x,y,w,h = cv.boundingRect(max_contour)
for i in contours:
  if(cv.contourArea(i) >= 5000):
    cv.drawContours(image, [i], 0, 255, 3)
    
cv.drawContours(image, [max_contour], 0, 255, 3)
cv.rectangle(image, (x,y), (x+w, y+h), (0,255,0),3)

#thresh = cv.adaptiveThreshold(gray, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 11, 2)
#hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
cv.imshow("image", image)
cv.imshow("image2", thresh)
cv.waitKey(0)
cv.destroyAllWindows()




