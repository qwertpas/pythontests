import cv2
import time
import numpy as np
from random import randrange

counter = 0
cap = cv2.VideoCapture(0, cv2.CAP_V4L)

while counter < 1000:
    _,frame = cap.read()
    print(time.time() + frame[10,10])
    counter+=1
    
# print(time.time() - last)
