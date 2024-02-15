import cv2 as cv

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
parameters =  cv.aruco.DetectorParameters()
detector = cv.aruco.ArucoDetector(dictionary, parameters)

frame = cv.imread(...)

markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(frame)