import cv2 as cv

cam = cv.VideoCapture(0)
result, image = cam.read()
if result:
    cv.imwrite("test.png", image)
else:
    print("No image detected. Please! try again")