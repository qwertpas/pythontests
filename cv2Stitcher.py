# import the necessary packages

import cv2
import numpy as np
 

cam0 = cv2.VideoCapture(0)

while(True):

    ret, img = (cam0.read())


    images = []

    print (img.shape)


    crop0 = img[0:720, 0:840]
    crop1 = img[0:720, 440:1280]


    images.append(crop0)
    images.append(crop1)

    cv2.imshow("crop0", crop0)  
    cv2.imshow("crop1", crop1)

    stitcher = cv2.Stitcher_create()
    (status, stitched) = stitcher.stitch(images)
    

    if status == 0:
        cv2.imshow("Stitched", stitched)
    
    # otherwise the stitching failed, likely due to not enough keypoints)
    # being detected
    else:
        print("[INFO] image stitching failed ({})".format(status))

    cv2.waitKey(1)  

    '''
    # initialize OpenCV's image sticher object and then perform the image
    # stitching
    print("[INFO] stitching images...")
    stitcher = cv2.Stitcher_create()
    (status, stitched) = stitcher.stitch(images)

    # if the status is '0', then OpenCV successfully performed image
    # stitching
    if status == 0:
        cv2.imshow("Stitched", stitched)
        cv2.waitKey(1)
    
    # otherwise the stitching failed, likely due to not enough keypoints)
    # being detected
    else:
        print("[INFO] image stitching failed ({})".format(status))
    '''